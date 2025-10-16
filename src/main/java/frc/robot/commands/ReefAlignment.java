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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  // Align to the barge on the current side of the field, don't cross barge with arm up
  private static final Distance bargeCenterX = Meters.of(8.774176);
  private static final Distance bargeAlignXOffset = Meters.of(0.65);

  // Only align to the current alliance barge, don't score for the opposing team
  private static final Distance bargeCenterY = Meters.of(4.0259);
  private static final Distance bargeAlignYOffset = Meters.of(0.5);

  private static final Distance blueBargeAlignYMin = bargeCenterY.plus(bargeAlignYOffset);
  private static final Distance redBargeAlignYMax = bargeCenterY.minus(bargeAlignYOffset);

  private static final Translation2d poleOffset = new Translation2d(Meters.zero(), Inches.of(6.8));

  private ReefAlignment() {}

  private static boolean isBlueSide(Pose2d robotPose) {
    return robotPose.getX() < 8.774176;
  }

  private static Pose2d findClosestReef(Pose2d robotPose) {
    // Get the correct reef based on which side of the field the robot is on
    Translation2d reefCenter = isBlueSide(robotPose) ? blueReefCenter : redReefCenter;
    double rotationsToReefCenter =
        robotPose.getTranslation().minus(reefCenter).getAngle().getRotations();

    // Round to nearest 1/6th rotation
    rotationsToReefCenter = Math.round(rotationsToReefCenter * 6) / 6.0;
    Rotation2d angleToReefCenter = Rotation2d.fromRotations(rotationsToReefCenter);

    return new Pose2d(reefCenter, angleToReefCenter);
  }

  private static Pose2d findBargeAlignPose(Pose2d robotPose) {
    Distance xMeters =
        bargeCenterX.plus(
            isBlueSide(robotPose) ? bargeAlignXOffset.unaryMinus() : bargeAlignXOffset);

    Distance yMeters = robotPose.getMeasureY();

    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
      if (yMeters.lt(blueBargeAlignYMin)) {
        yMeters = blueBargeAlignYMin;
      }
    } else {
      if (yMeters.gt(redBargeAlignYMax)) {
        yMeters = redBargeAlignYMax;
      }
    }

    Rotation2d rotation = isBlueSide(robotPose) ? Rotation2d.k180deg : Rotation2d.kZero;

    return new Pose2d(xMeters, yMeters, rotation);
  }

  private static Pose2d[] findReefPolePoses(Pose2d robotPose, Distance reefOffset) {
    Pose2d closestReef = findClosestReef(robotPose);
    return new Pose2d[] {
      closestReef.transformBy(
          new Transform2d(
              new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero())
                  .plus(poleOffset)
                  .plus(robotRelatveOffset),
              Rotation2d.k180deg)),
      closestReef.transformBy(
          new Transform2d(
              new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero())
                  .minus(poleOffset)
                  .plus(robotRelatveOffset),
              Rotation2d.k180deg)),
    };
  }

  private static Pose2d[] findCenteredReefPose(
      Pose2d robotPose, Distance reefOffset, Rotation2d rotation) {
    Pose2d closestReef = findClosestReef(robotPose);
    return new Pose2d[] {
      closestReef.transformBy(
          new Transform2d(
              new Translation2d(reefBaseOffsetDistance.plus(reefOffset), Meters.zero()), rotation)),
    };
  }

  private static Pose2d[] findCenteredReefPose(Pose2d robotPose, Distance reefOffset) {
    return findCenteredReefPose(robotPose, reefOffset, Rotation2d.k180deg);
  }

  private static Pose2d[] getAlignPoses(Pose2d robotPose, ScoringPosition position) {
    switch (position) {
      case troughL1:
        return findCenteredReefPose(robotPose, reefLowerOffsetDistance);
      case branchL2, branchL3:
        return findReefPolePoses(robotPose, reefLowerOffsetDistance);
      case branchL4:
        return findReefPolePoses(robotPose, reefL4OffsetDistance);

      case algaeL2, algaeL3:
        return findCenteredReefPose(robotPose, reefAlgaeOffsetDistance);

      case barge:
        return new Pose2d[] {findBargeAlignPose(robotPose)};

      default:
        return new Pose2d[0];
    }
  }

  private static Optional<Pose2d> findAlignPose(Pose2d robotPose, ScoringPosition position) {
    Pose2d[] alignPoses = getAlignPoses(robotPose, position);

    Optional<Pose2d> closestOffset =
        Arrays.stream(alignPoses)
            .map(
                (alignPose) -> {
                  double offsetDistance =
                      robotPose.getTranslation().getDistance(alignPose.getTranslation());

                  return new Pair<>(alignPose, offsetDistance);
                })
            .min((a, b) -> Double.compare(a.getSecond(), b.getSecond()))
            .map((pair) -> pair.getFirst());

    return closestOffset;
  }

  private static ChassisSpeeds getAutoAlignSpeeds(
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

  public static ChassisSpeeds getAutoAlignSpeeds(Pose2d alignPose, DrivePid driveController) {
    return getAutoAlignSpeeds(alignPose, driveController, 0);
  }

  public static Command driveAutoAligned(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier,
      Supplier<ScoringPosition> positionSupplier) {

    DrivePid driveController = drive.getPid();

    return Commands.run(
            () -> {
              Optional<Pose2d> closestPose = findAlignPose(drive.getPose(), positionSupplier.get());

              double joystickValue = Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble());

              ChassisSpeeds speeds;

              if (closestPose.isPresent()) {
                Pose2d foundPose = closestPose.get();

                speeds =
                    DriveCommands.getJoystickSpeeds(
                        drive, xSupplier.getAsDouble(), ySupplier.getAsDouble(), 0);

                speeds =
                    speeds.plus(
                        ChassisSpeeds.fromFieldRelativeSpeeds(
                            getAutoAlignSpeeds(foundPose, driveController, joystickValue),
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
