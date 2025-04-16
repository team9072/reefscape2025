package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.scoring.ScoringPosition;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DrivePid;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class ReefAlignment {
  private static final Distance reefLowerOffsetDistance = Inches.of(7.5);
  private static final Distance reefL4OffsetDistance = Inches.of(10.25);
  private static final Distance reefAlgaeOffsetDistance = Inches.of(15);

  private static final Translation2d robotRelatveOffset =
      new Translation2d(Meters.zero(), Inches.of(-1.25));

  private static final Translation2d blueReefCenter = new Translation2d(4.489323, 4.0259);
  private static final Translation2d redReefCenter = new Translation2d(13.058902, 4.0259);

  // Make the front of the robot go to the tag, not the center
  private static final Distance reefTagOffsetDistance = Meters.of(0.831723);
  private static final Distance reefBumperOffsetDistance = Inches.of(16);
  private static final Distance reefBaseOffsetDistance =
      reefTagOffsetDistance.plus(reefBumperOffsetDistance);

  private static final Translation2d poleOffset = new Translation2d(Meters.zero(), Inches.of(6.8));

  private ReefAlignment() {}

  private static Transform2d[] getPoleOffsets(Distance reefOffset) {
    return new Transform2d[] {
      new Transform2d(
          new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero())
              .plus(poleOffset)
              .plus(robotRelatveOffset),
          Rotation2d.k180deg),
      new Transform2d(
          new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero())
              .minus(poleOffset)
              .plus(robotRelatveOffset),
          Rotation2d.k180deg),
    };
  }

  private static Transform2d[] getSingleOffset(Distance reefOffset, Rotation2d rotation) {
    return new Transform2d[] {
      new Transform2d(
          new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero()), rotation),
    };
  }

  private static Transform2d[] getSingleOffset(Distance reefOffset) {
    return getSingleOffset(reefOffset, Rotation2d.k180deg);
  }

  private static Transform2d[] getReefOffsets(ScoringPosition position) {
    switch (position) {
      case troughL1:
        return getSingleOffset(reefLowerOffsetDistance);
      case branchL2, branchL3:
        return getPoleOffsets(reefLowerOffsetDistance);
      case branchL4:
        return getPoleOffsets(reefL4OffsetDistance);

      case algaeL2, algaeL3:
        return getSingleOffset(reefAlgaeOffsetDistance);

      default:
        return new Transform2d[0];
    }
  }

  private static Pose2d findClosestReef(Pose2d robotPose) {
    // Get the correct reef based on which side of the field the robot is on
    Translation2d reefCenter = robotPose.getX() < 8.774176 ? blueReefCenter : redReefCenter;
    double rotationsToReefCenter =
        robotPose.getTranslation().minus(reefCenter).getAngle().getRotations();

    // Round to nearest 1/6th rotation
    rotationsToReefCenter = Math.round(rotationsToReefCenter * 6) / 6.0;
    Rotation2d angleToReefCenter = Rotation2d.fromRotations(rotationsToReefCenter);

    return new Pose2d(reefCenter, angleToReefCenter);
  }

  private static Optional<Pose2d> findClosestOffset(
      Pose2d robotPose, Pose2d reefPose, ScoringPosition position) {
    Transform2d[] reefOffsets = getReefOffsets(position);
    Optional<Pose2d> closestOffset =
        Arrays.stream(reefOffsets)
            .map(
                (offset) -> {
                  Pose2d offsetPose = reefPose.transformBy(offset);
                  double offsetDistance =
                      robotPose.getTranslation().getDistance(offsetPose.getTranslation());

                  return new Pair<>(offsetPose, offsetDistance);
                })
            .min((a, b) -> Double.compare(a.getSecond(), b.getSecond()))
            .map((pair) -> pair.getFirst());

    return closestOffset;
  }

  private static ChassisSpeeds getReefAlignSpeeds(
      Pose2d alignPose, DrivePid driveController, double joystickValue) {
    ChassisSpeeds speeds = new ChassisSpeeds();

    ChassisSpeeds headingCorrection = driveController.getHeadingCorrection(alignPose.getRotation());

    if (!driveController.headingPidAtSetpoint()) {
      speeds = speeds.plus(headingCorrection);
    }

    ChassisSpeeds positionCorrection =
        driveController
            .getTranslationCorrection(alignPose.getTranslation())
            .times(Math.max(0, 1 - joystickValue * 200));

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
      DoubleSupplier omegaSupplier,
      Supplier<ScoringPosition> positionSupplier) {

    DrivePid driveController = drive.getPid();

    return Commands.run(
            () -> {
              Pose2d closestReef = findClosestReef(drive.getPose());
              Optional<Pose2d> closestOffset =
                  findClosestOffset(drive.getPose(), closestReef, positionSupplier.get());

              double joystickValue = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              ChassisSpeeds speeds;

              if (closestOffset.isPresent()) {
                speeds =
                    DriveCommands.getJoystickSpeeds(
                        drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);

                speeds =
                    speeds.plus(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            // If no offset was found, just align to the reef
                            getReefAlignSpeeds(
                                closestOffset.orElse(drive.getPose()),
                                driveController,
                                joystickValue),
                            drive.getRotation()));
              } else {
                speeds =
                    DriveCommands.getJoystickSpeeds(
                        drive,
                        xSupplier.getAsDouble(),
                        ySupplier.getAsDouble(),
                        omegaSupplier.getAsDouble());
              }

              drive.runVelocity(speeds);
            },
            drive)
        .withName("AlignToReef");
  }
}
