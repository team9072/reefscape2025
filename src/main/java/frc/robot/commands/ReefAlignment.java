package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.CoralFlow.ReefBranch;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DrivePid;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ReefAlignment {
  private static final Distance reefLowerOffsetDistance = Inches.of(6.5);
  private static final Distance reefL4OffsetDistance = Inches.of(8);

  private static final Translation2d blueReefCenter = new Translation2d(4.489323, 4.0259);
  private static final Translation2d redReefCenter = new Translation2d(13.058902, 4.0259);

  // Make the front of the robot go to the tag, not the center
  private static final Distance reefTagOffsetDistance = Meters.of(0.831723);
  private static final Distance reefBumperOffsetDistance = Inches.of(16);
  private static final Distance reefBaseOffsetDistance =
      reefTagOffsetDistance.plus(reefBumperOffsetDistance);

  private static final Translation2d poleOffset = new Translation2d(Meters.zero(), Inches.of(6.8));

  private ReefAlignment() {}

  private static Translation2d getReefOffset(ReefBranch branch) {
    Distance reefOffset =
        switch (branch) {
          case branchL2, branchL3:
            yield reefLowerOffsetDistance;
          case branchL4:
            yield reefL4OffsetDistance;
        };

    return new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero());
  }

  private static Pose2d findClosestReef(Pose2d robotPose, ReefBranch branch) {
    // Get the correct reef based on which side of the field the robot is on
    Translation2d reefCenter = robotPose.getX() < 8.774176 ? blueReefCenter : redReefCenter;
    double rotationsToReefCenter =
        robotPose.getTranslation().minus(reefCenter).getAngle().getRotations();

    // Round to nearest 1/6th rotation
    rotationsToReefCenter = Math.round(rotationsToReefCenter * 6) / 6.0;
    Rotation2d angleToReefCenter = Rotation2d.fromRotations(rotationsToReefCenter);

    Translation2d reefSide = reefCenter.plus(getReefOffset(branch).rotateBy(angleToReefCenter));

    return new Pose2d(reefSide, angleToReefCenter.rotateBy(Rotation2d.k180deg));
  }

  private static Pose2d findClosestPole(Pose2d robotPose, Pose2d reefPose) {
    Translation2d reefSide = reefPose.getTranslation();
    Translation2d poleOffset = ReefAlignment.poleOffset.rotateBy(reefPose.getRotation());

    Translation2d pole1 = reefSide.plus(poleOffset);
    Translation2d pole2 = reefSide.minus(poleOffset);

    Translation2d closestPole =
        (pole1.getDistance(robotPose.getTranslation())
                < pole2.getDistance(robotPose.getTranslation()))
            ? pole1
            : pole2;

    return new Pose2d(closestPole, reefPose.getRotation());
  }

  private static ChassisSpeeds getReefAlignSpeeds(
      Pose2d reefPole, DrivePid driveController, double joystickValue) {
    ChassisSpeeds speeds = new ChassisSpeeds();

    ChassisSpeeds headingCorrection = driveController.getHeadingCorrection(reefPole.getRotation());

    if (!driveController.headingPidAtSetpoint()) {
      speeds = speeds.plus(headingCorrection);
    }

    ChassisSpeeds positionCorrection =
        driveController
            .getTranslationCorrection(reefPole.getTranslation())
            .times(Math.max(0, 1 - joystickValue * 10));

    if (!driveController.translationPidAtSetpoint()) {
      speeds = speeds.plus(positionCorrection);
    }

    return speeds;
  }

  public static ChassisSpeeds getReefAlignSpeeds(Pose2d reefPole, DrivePid driveController) {
    return getReefAlignSpeeds(reefPole, driveController, 0);
  }

  public static Command driveReefAligned(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<ReefBranch> branchSupplier) {

    DrivePid driveController = drive.getPid();

    return Commands.run(
        () -> {
          Pose2d closestReef = findClosestReef(drive.getPose(), branchSupplier.get());
          Pose2d closestPole = findClosestPole(drive.getPose(), closestReef);

          double joystickValue = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          ChassisSpeeds speeds =
              DriveCommands.getJoystickSpeeds(
                  drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);

          drive.runVelocity(
              speeds.plus(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      getReefAlignSpeeds(closestPole, driveController, joystickValue),
                      drive.getRotation())));
        },
        drive);
  }
}
