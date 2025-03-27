package frc.robot.commands.autos.chooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autos.AutoTrajectories.PreloadTrajectory;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class AutoBuilder extends LoggedNetworkInput {
  private final String key;
  private final Field2d autoDisplay = new Field2d();

  private final TrajectoryChooser<PreloadTrajectory> preloadChooser =
      new TrajectoryChooser<>(PreloadTrajectory.values());
  private final String preloadKey;
  private PreloadTrajectory preloadTrajectory;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(preloadKey, preloadTrajectory);
        }

        public void fromLog(LogTable table) {
          preloadTrajectory = table.get(preloadKey, preloadTrajectory);
        }
      };

  private static boolean flipTrajectoryDisplay() {
    return DriverStation.getAlliance().map((alliance) -> alliance == Alliance.Red).orElse(false);
  }

  public AutoBuilder(String key) {
    this.key = key;
    preloadKey = getKey("preload");
    SmartDashboard.putData(preloadKey, preloadChooser);
    SmartDashboard.putData(getKey("field"), autoDisplay);
    periodic();
    Logger.registerDashboardInput(this);
  }

  private String getKey(String path) {
    return key + "/" + path;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      preloadTrajectory = preloadChooser.getSelected();
    }

    Logger.processInputs(prefix + "/SmartDashboard", inputs);

    Pose2d startingPose = Pose2d.kZero;
    if (preloadTrajectory != null) {
      startingPose =
          preloadTrajectory.trajectory.getInitialPose(flipTrajectoryDisplay()).orElse(startingPose);
      autoDisplay.setRobotPose(startingPose);
      autoDisplay.getObject(preloadKey).setPoses(preloadTrajectory.trajectory.getPoses());
    } else {
      autoDisplay.setRobotPose(Pose2d.kZero);
      autoDisplay.getObject(preloadKey).setPoses();
    }
  }
}
