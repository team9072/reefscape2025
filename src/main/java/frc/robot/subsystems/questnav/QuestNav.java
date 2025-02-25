package frc.robot.subsystems.questnav;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class QuestNav extends SubsystemBase {
  private final QuestNavIO questnavIO;
  private final QuestNavInputsAutoLogged questnavInputs = new QuestNavInputsAutoLogged();

  public QuestNav(QuestNavIO questnavIO) {
    this.questnavIO = questnavIO;
  }

  @Override
  public void periodic() {
    questnavIO.updateInputs(questnavInputs);
    Logger.processInputs("QuestNav", questnavInputs);
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
}
