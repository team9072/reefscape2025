package frc.robot.commands.autos.sequences;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.scoring.ScoringPosition;
import frc.robot.subsystems.drive.Drive.DrivePid;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import java.util.concurrent.atomic.AtomicBoolean;

public class AutoSequences {
  private final AutoFactory autoFactory;
  private final Robot.Subsystems s;

  private final DrivePid driveController;

  public AutoSequences(Robot.Subsystems s) {
    driveController = s.drive.getPid();

    this.autoFactory =
        new AutoFactory(
            s.drive::getPose,
            (pose) -> {
              s.drive.setPose(pose);
              s.questNav.resetPose(pose);
            },
            this::drivePath,
            true,
            s.drive);
    this.s = s;
  }

  public AutoFactory getFactory() {
    return autoFactory;
  }

  private void drivePath(SwerveSample sample) {
    ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();

    Pose2d samplePose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));

    targetSpeeds = targetSpeeds.plus(driveController.getPoseCorrection(samplePose));

    s.drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, s.drive.getRotation()));
  }

  private Command drivePose(Pose2d targetPose) {
    return Commands.run(
            () -> {
              ChassisSpeeds targetSpeeds =
                  ReefAlignment.getReefAlignSpeeds(targetPose, driveController);

              s.drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, s.drive.getRotation()));
            },
            s.drive)
        .until(() -> driveController.posePidAtSetpoint(targetPose));
  }

  private Command completeAlign(AutoTrajectory trajectory) {
    return drivePose(trajectory.getFinalPose().get());
  }

  private Command pathCommandOrNone(AutoTrajectory nullableTrajectory) {
    if (nullableTrajectory == null) {
      return Commands.none();
    }

    return nullableTrajectory.cmd();
  }

  private Command scoreAtEnd(
      AutoTrajectory scoreTrajectory, ScoringPosition position, AutoTrajectory nextTrajectory) {
    return Commands.sequence(
        completeAlign(scoreTrajectory).deadlineFor(s.scoring.prepareElevatorAuto(position)),
        s.scoring
            .scoringActionAuto(position)
            .deadlineFor(completeAlign(scoreTrajectory).repeatedly()),
        Commands.parallel(s.scoring.elevatorDown(), pathCommandOrNone(nextTrajectory)));
  }

  public void scorePreload(
      ScoringPosition scoringPosition,
      AutoTrajectory preloadTrajectory,
      AutoTrajectory nextTrajectory) {
    preloadTrajectory.active().onTrue(s.scoring.stowAlgae());

    preloadTrajectory
        .atTimeBeforeEnd(0.5)
        .onTrue(
            s.scoring
                .prepareElevatorAuto(scoringPosition)
                .alongWith(s.intake.setPosition(PivotPosition.deploy)));

    preloadTrajectory.done().onTrue(scoreAtEnd(preloadTrajectory, scoringPosition, nextTrajectory));
  }

  public void intakeAndScore(
      AutoRoutine routine,
      ScoringPosition scoringPosition,
      AutoTrajectory intakeTrajectory,
      AutoTrajectory prepareTrajectory,
      AutoTrajectory scoreTrajectory,
      AutoTrajectory nextTrajectory) {
    final AtomicBoolean stagedCoral = new AtomicBoolean(false);
    final AtomicBoolean grabbedCoral = new AtomicBoolean(false);

    routine
        .active()
        .onTrue(
            Commands.runOnce(
                () -> {
                  stagedCoral.set(false);
                  grabbedCoral.set(false);
                }));

    intakeTrajectory
        .active()
        .onTrue(
            Commands.sequence(
                s.intake.setPosition(PivotPosition.deploy).withTimeout(0), s.intake.intake()));

    // If still has coral from miss last piece, just go score
    intakeTrajectory
        .active()
        .and(s.scoring.coralPlacerHasObject)
        .and(() -> !stagedCoral.get())
        .onTrue(
            Commands.sequence(
                Commands.runOnce(
                    () -> {
                      stagedCoral.set(true);
                      grabbedCoral.set(true);
                    }),
                prepareTrajectory.cmd()));

    intakeTrajectory
        .active()
        .and(s.intake.coralInPassthrough.or(s.intake.coralStaged))
        .onTrue(prepareTrajectory.cmd());
    intakeTrajectory.chain(prepareTrajectory);

    prepareTrajectory
        .recentlyDone()
        .or(routine.anyActive(intakeTrajectory, prepareTrajectory))
        .and(s.intake.coralStaged)
        .and(() -> !(grabbedCoral.get() || stagedCoral.get()))
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> stagedCoral.set(true)),
                s.scoring.grabCoral(),
                Commands.runOnce(() -> grabbedCoral.set(true))));

    prepareTrajectory
        .recentlyDone()
        .debounce(1.5)
        .and(() -> !stagedCoral.get())
        .and(() -> nextTrajectory != null)
        .onTrue(pathCommandOrNone(nextTrajectory));

    prepareTrajectory
        .recentlyDone()
        .and(grabbedCoral::get)
        .onTrue(scoreTrajectory.cmd().deadlineFor(s.scoring.prepareElevatorAuto(scoringPosition)));

    scoreTrajectory.done().onTrue(scoreAtEnd(scoreTrajectory, scoringPosition, nextTrajectory));
  }
}
