package frc.robot.commands.autos.sequences;

import choreo.auto.AutoFactory;
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

public class AutoSequences {
  private static final String prepareAlignEvent = "PrepareAlign";

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

  public void scorePreload(AutoTrajectory trajectory, AutoTrajectory nextPath) {
    trajectory.active().onTrue(s.scoring.stowHold());

    trajectory
        .atTime(prepareAlignEvent)
        .onTrue(s.scoring.prepareElevator(ScoringPosition.branchL4));

    trajectory
        .done()
        .onTrue(
            Commands.sequence(
                completeAlign(trajectory),
                s.scoring.scoringActionAuto(ScoringPosition.branchL4),
                Commands.parallel(s.scoring.elevatorDown(), pathCommandOrNone(nextPath))));
  }

  public void intakeAndScore(
      AutoTrajectory intakeTrajectory,
      AutoTrajectory prepareTrajectory,
      AutoTrajectory scoreTrajectory,
      AutoTrajectory nextPath) {
    intakeTrajectory
        .active()
        .onTrue(
            s.intake.setPosition(PivotPosition.deploy).withTimeout(0).andThen(s.intake.intake()));

    intakeTrajectory
        .active()
        .and(s.intake.coralInPassthrough.or(s.intake.coralStaged))
        .onTrue(prepareTrajectory.cmd());
    intakeTrajectory.chain(prepareTrajectory);

    prepareTrajectory
        .done()
        .onTrue(
            Commands.sequence(
                s.scoring.grabCoral(),
                s.scoring.prepareElevator(ScoringPosition.branchL4),
                scoreTrajectory.cmd()));

    scoreTrajectory
        .recentlyDone()
        .onTrue(
            Commands.sequence(
                completeAlign(scoreTrajectory),
                s.scoring.scoringActionAuto(ScoringPosition.branchL4),
                Commands.parallel(s.scoring.elevatorDown(), pathCommandOrNone(nextPath))));
  }
}
