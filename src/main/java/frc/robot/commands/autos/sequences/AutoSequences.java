package frc.robot.commands.autos.sequences;

import static edu.wpi.first.units.Units.Degrees;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

  private final Rotation2d reefSafeAngle = new Rotation2d(Degrees.of(40));
  private Rotation2d rotationOverride = null;

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

  private Trigger anyActive(AutoTrajectory trajectory, AutoTrajectory... trajectories) {
    Trigger trigger = trajectory.active();
    for (int i = 0; i < trajectories.length; i++) {
      trigger = trigger.or(trajectories[i].active());
    }
    return trigger;
  }

  private void drivePath(SwerveSample sample) {
    ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();

    Pose2d samplePose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
    if (rotationOverride != null) {
      samplePose = new Pose2d(samplePose.getTranslation(), rotationOverride);
    }

    targetSpeeds = targetSpeeds.plus(driveController.getPoseCorrection(samplePose));

    s.drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, s.drive.getRotation()));
  }

  private Command setRotationOverride(Rotation2d override) {
    return Commands.runOnce(() -> rotationOverride = override);
  }

  private Command clearRotationOverride() {
    return setRotationOverride(null);
  }

  private Command overrideFinalPose(AutoTrajectory trajectory, Rotation2d offset) {
    return setRotationOverride(trajectory.getFinalPose().get().getRotation().plus(offset));
  }

  private Command drivePose(Pose2d targetPose) {
    return Commands.run(
            () -> {
              Pose2d currentTargetPose = targetPose;

              if (rotationOverride != null) {
                currentTargetPose =
                    new Pose2d(currentTargetPose.getTranslation(), rotationOverride);
              }

              ChassisSpeeds targetSpeeds =
                  ReefAlignment.getReefAlignSpeeds(currentTargetPose, driveController);

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
    Command baseCommand = clearRotationOverride();

    if (nullableTrajectory == null) {
      return baseCommand;
    }

    return baseCommand.alongWith(nullableTrajectory.cmd());
  }

  private Command scoreAtEnd(
      AutoTrajectory scoreTrajectory, ScoringPosition position, AutoTrajectory nextTrajectory) {
    return Commands.sequence(
        clearRotationOverride(),
        completeAlign(scoreTrajectory),
        s.scoring.scoringActionAuto(position),
        Commands.parallel(s.scoring.elevatorDown(), pathCommandOrNone(nextTrajectory)));
  }

  private Command grabAtSafeRotation(AutoTrajectory scoreTrajectory) {
    return Commands.sequence(
        overrideFinalPose(scoreTrajectory, reefSafeAngle),
        s.scoring.grabCoral(),
        clearRotationOverride());
  }

  public void scorePreload(
      ScoringPosition scoringPosition,
      AutoTrajectory preloadTrajectory,
      AutoTrajectory nextTrajectory) {
    preloadTrajectory.active().onTrue(s.scoring.stowHold());

    preloadTrajectory.atTimeBeforeEnd(0.5).onTrue(s.scoring.prepareElevatorAuto(scoringPosition));

    preloadTrajectory.done().onTrue(scoreAtEnd(preloadTrajectory, scoringPosition, nextTrajectory));
  }

  public void intakeAndScore(
      ScoringPosition scoringPosition,
      AutoTrajectory intakeTrajectory,
      AutoTrajectory scoreTrajectory,
      AutoTrajectory nextTrajectory) {
    final AtomicBoolean stagedCoral = new AtomicBoolean(false);
    final AtomicBoolean grabbedCoral = new AtomicBoolean(false);

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
                scoreTrajectory.cmd()));

    intakeTrajectory
        .active()
        .and(s.intake.coralInPassthrough.or(s.intake.coralStaged))
        .onTrue(scoreTrajectory.cmd());
    intakeTrajectory.chain(scoreTrajectory);

    anyActive(intakeTrajectory, scoreTrajectory)
        .and(s.intake.coralStaged)
        .and(() -> !(grabbedCoral.get() || stagedCoral.get()))
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> stagedCoral.set(true)),
                grabAtSafeRotation(scoreTrajectory),
                Commands.runOnce(() -> grabbedCoral.set(true))));

    scoreTrajectory.active().onTrue(overrideFinalPose(scoreTrajectory, reefSafeAngle));

    scoreTrajectory.done().and(() -> !stagedCoral.get()).onTrue(pathCommandOrNone(nextTrajectory));

    scoreTrajectory
        .recentlyDone()
        .and(grabbedCoral::get)
        .onTrue(scoreAtEnd(scoreTrajectory, scoringPosition, nextTrajectory));
  }
}
