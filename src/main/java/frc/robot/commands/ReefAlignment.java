package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class ReefAlignment {
  private static final Translation2d blueReefCenter = new Translation2d(4.489323, 4.0259);
  private static final Translation2d redReefCenter = new Translation2d(13.058902, 4.0259);
  private static final Translation2d reefOffset = new Translation2d(0.831723, 0);

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

          ChassisSpeeds speeds =
              DriveCommands.getJoystickSpeeds(
                  drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);
          drive.addHeadingCorrection(speeds, closestReef.getRotation());

          drive.runVelocity(speeds);
        },
        drive);
  }
}
