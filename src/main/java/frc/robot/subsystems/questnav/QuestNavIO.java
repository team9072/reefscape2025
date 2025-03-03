package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import org.littletonrobotics.junction.AutoLog;

public interface QuestNavIO {
  @AutoLog
  public static class QuestNavInputs {
    public QuestNavPoseObservation latestObservation = new QuestNavPoseObservation(new Pose2d(), 0);
    public Time latestTimestamp = Seconds.zero();
    public long frameCount;
    public double batteryPercentage;
    public boolean isBusy;
  }

  public static record QuestNavPoseObservation(Pose2d estimatedRobotPose, double timestamp) {}

  public default void updateInputs(QuestNavInputs inputs) {}

  public default void resetPose(Pose2d robotPose) {}
}
