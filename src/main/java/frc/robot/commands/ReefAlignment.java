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
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ReefAlignment {

  private static final Distance faceOffsetDistance = Inches.of(16);

  private static final Translation2d blueReefCenter = new Translation2d(4.489323, 4.0259);
  private static final Translation2d redReefCenter = new Translation2d(13.058902, 4.0259);
  // Make the front of the robot go to the tag, not the center
  private static final Translation2d reefOffset =
      new Translation2d(Meters.of(0.831723).plus(faceOffsetDistance), Meters.zero());

  private static final Translation2d poleOffset = new Translation2d(Meters.zero(), Inches.of(6.5));

  private ReefAlignment() {}

  private static Pose2d findClosestReef(Pose2d robotPose) {
    // Get the correct reef based on which side of the field the robot is on
    Translation2d reefCenter = robotPose.getX() < 8.774176 ? blueReefCenter : redReefCenter;
    double rotationsToReefCenter =
        robotPose.getTranslation().minus(reefCenter).getAngle().getRotations();

    // Round to nearest 1/6th rotation
    rotationsToReefCenter = Math.round(rotationsToReefCenter * 6) / 6.0;
    Rotation2d angleToReefCenter = Rotation2d.fromRotations(rotationsToReefCenter);

    Translation2d reefSide = reefCenter.plus(reefOffset.rotateBy(angleToReefCenter));

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

  public static Command driveReefAligned(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          Pose2d closestReef = findClosestReef(drive.getPose());
          Logger.recordOutput("ReefAlign/Closest Side", closestReef);

          Pose2d closestPole = findClosestPole(drive.getPose(), closestReef);
          Logger.recordOutput("ReefAlign/Closest Pole", closestPole);

          double joystickValue = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          ChassisSpeeds speeds =
              DriveCommands.getJoystickSpeeds(
                  drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);

          ChassisSpeeds headingCorrection = drive.getHeadingCorrection(closestPole.getRotation());

          if (!drive.headingPidAtSetpoint()) {
            speeds = speeds.plus(headingCorrection);
          }

          ChassisSpeeds positionCorrection =
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  drive
                      .getTranslationCorrection(closestPole.getTranslation())
                      .times(Math.max(0, 1 - (Math.sqrt(joystickValue)))),
                  drive.getRotation());

          if (!drive.positionPidAtSetpoint()) {
            speeds = speeds.plus(positionCorrection);
          }

          drive.runVelocity(speeds);
        },
        drive);
  }
}
