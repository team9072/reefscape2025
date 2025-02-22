package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class ReefAlignment {
  private static final Translation2d blueReefCenter = new Translation2d(4.489323, 4.0259);
  private static final Translation2d redReefCenter = new Translation2d(13.058902, 4.0259);
  // Make the front of the robot go to the tag, not the center
  private static final Translation2d reefOffset =
      new Translation2d(Meters.of(0.831723).plus(Inches.of(16)), Meters.zero());

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

  public static Command driveReefAligned(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {

    return Commands.run(
        () -> {
          Pose2d closestReef = findClosestReef(drive.getPose());
          Logger.recordOutput("ReefAlignPose", closestReef);

          double joystickValue = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          ChassisSpeeds speeds =
              DriveCommands.getJoystickSpeeds(
                      drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0)
                  .plus(drive.getHeadingCorrection(closestReef.getRotation()))
                  .plus(
                      ChassisSpeeds.fromFieldRelativeSpeeds(
                          drive
                              .getTranslationCorrection(closestReef.getTranslation())
                              .div(1 + (joystickValue * 2)),
                          drive.getRotation()));

          drive.runVelocity(speeds);
        },
        drive);
  }
}
