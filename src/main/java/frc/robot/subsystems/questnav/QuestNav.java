package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {
  private final PositionConsumer positionConsumer;
  private final QuestNavIO questnavIO;
  private final QuestNavInputsAutoLogged questnavInputs = new QuestNavInputsAutoLogged();

  private long lastFrameCount = 0;

  public QuestNav(QuestNavIO questnavIO, PositionConsumer positionConsumer) {
    this.questnavIO = questnavIO;
    this.positionConsumer = positionConsumer;
  }

  @Override
  public void periodic() {
    questnavIO.updateInputs(questnavInputs);
    Logger.processInputs("QuestNav", questnavInputs);

    // Keep odometry rotation
    Translation2d robotPosition =
        questnavInputs.latestObservation.estimatedRobotPose().getTranslation();

    // Don't send poses if the quest hasn't updated (eg. it's disconnected)
    if (questnavInputs.frameCount > lastFrameCount) {
      lastFrameCount = questnavInputs.frameCount;

      positionConsumer.accept(
          robotPosition,
          questnavInputs.latestObservation.timestamp(),
          VecBuilder.fill(
              QuestNavConstants.translationStdDevs.in(Meters),
              QuestNavConstants.translationStdDevs.in(Meters),
              QuestNavConstants.rotationStdDevs.in(Radians)));
    }
  }

  /**
   * Updates the Quest Nav software about where the Quest currently is relative to the field.
   * Running this function is critically important to the Quest Nav's ability to tell the robot
   * where it is on the field accurately.
   *
   * <p>There are two methods of handling the use of this function correctly:
   *
   * <ul>
   *   <li>Position the robot on the field in a known position, and using that known position when
   *       resetting the pose. This could potentially be done during field setup by the drive team,
   *       using a button that is detected while the robot is on and disabled on the field.
   *   <li>Utilizing a trustworthy source of the robot's absolute position and orientation on the
   *       field, like a April Tag tracking system.
   * </ul>
   *
   * @param poseOfRobotOnField
   * @return true if the request was actually sent, false if the device is busy
   */
  public boolean resetPose(Pose2d robotPose) {
    if (questnavInputs.isBusy) {
      System.err.println("QuestNav reset position attempt ignored. QuestNav is busy.");
      return false;
    }

    questnavIO.resetPose(robotPose);

    return true;
  }

  @FunctionalInterface
  public static interface PositionConsumer {
    public void accept(
        Translation2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
