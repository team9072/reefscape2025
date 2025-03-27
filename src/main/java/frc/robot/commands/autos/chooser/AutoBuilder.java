package frc.robot.commands.autos.chooser;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autos.AutoTrajectories.AutoTrajectory;
import frc.robot.commands.autos.AutoTrajectories.IntakeTrajectory;
import frc.robot.commands.autos.AutoTrajectories.PreloadTrajectory;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class AutoBuilder extends LoggedNetworkInput {
  private static class ManagedChooser<V extends AutoTrajectory> {
    public final String key;
    private final TrajectoryChooser<V> chooser;
    public V selectedTrajectory;
    private AutoTrajectory priorTrajectory;

    public ManagedChooser(String key, V[] trajectories, ManagedChooser<?> priorChooser) {
      this.key = key;
      this.chooser = new TrajectoryChooser<V>(trajectories);
      if (priorChooser != null) {
        updateFilters(priorChooser, true);
      }

      SmartDashboard.putData(key, chooser);
    }

    public void updateSelected() {
      selectedTrajectory = chooser.getSelected();
    }

    public void updateFilters(ManagedChooser<?> priorChooser, boolean force) {
      if (!force && priorChooser.selectedTrajectory == priorTrajectory) {
        return;
      }

      this.priorTrajectory = priorChooser.selectedTrajectory;
      if (priorTrajectory == null) {
        chooser.filterOptions((trajectory) -> false);
      } else {
        chooser.filterOptions(
            (trajectory) -> trajectory.initialPosition() == priorTrajectory.finalPosition());
      }
    }

    public void updateFilters(ManagedChooser<?> priorChooser) {
      updateFilters(priorChooser, false);
    }

    public void updateDisplay(Field2d autoDisplay, boolean setRobotPose) {
      if (setRobotPose) {
        Pose2d startingPose = Pose2d.kZero;
        if (selectedTrajectory != null) {
          startingPose =
              selectedTrajectory
                  .trajectory()
                  .getInitialPose(flipTrajectoryDisplay())
                  .orElse(startingPose);
        }

        autoDisplay.setRobotPose(startingPose);
      }

      if (selectedTrajectory != null) {
        Trajectory<SwerveSample> path = selectedTrajectory.trajectory();
        if (flipTrajectoryDisplay()) {
          path = path.flipped();
        }

        autoDisplay.getObject(key).setPoses(path.getPoses());
      } else {
        autoDisplay.getObject(key).setPoses();
      }
    }
  }

  private final String key;
  private final Field2d autoDisplay = new Field2d();

  private final ManagedChooser<PreloadTrajectory> preloadChooser;
  private final ManagedChooser<IntakeTrajectory> p2Chooser;
  private final ManagedChooser<IntakeTrajectory> p3Chooser;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        public void toLog(LogTable table) {
          table.put(preloadChooser.key, preloadChooser.selectedTrajectory);
          table.put(p2Chooser.key, p2Chooser.selectedTrajectory);
          table.put(p3Chooser.key, p3Chooser.selectedTrajectory);
        }

        public void fromLog(LogTable table) {
          preloadChooser.selectedTrajectory =
              table.get(preloadChooser.key, preloadChooser.selectedTrajectory);
          p2Chooser.selectedTrajectory = table.get(p2Chooser.key, p2Chooser.selectedTrajectory);
          p3Chooser.selectedTrajectory = table.get(p2Chooser.key, p2Chooser.selectedTrajectory);
        }
      };

  private static boolean flipTrajectoryDisplay() {
    return DriverStation.getAlliance().map((alliance) -> alliance == Alliance.Red).orElse(false);
  }

  public AutoBuilder(String key) {
    this.key = key;
    preloadChooser = new ManagedChooser<>(getKey("Preload"), PreloadTrajectory.values(), null);
    p2Chooser =
        new ManagedChooser<>(getKey("Second Piece"), IntakeTrajectory.values(), preloadChooser);
    p3Chooser = new ManagedChooser<>(getKey("Third Piece"), IntakeTrajectory.values(), p2Chooser);

    SmartDashboard.putData(getKey("Auto Path"), autoDisplay);
    periodic();
    Logger.registerDashboardInput(this);
  }

  private String getKey(String path) {
    return key + "/" + path;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      preloadChooser.updateSelected();
      p2Chooser.updateSelected();
      p3Chooser.updateSelected();
    }

    p2Chooser.updateFilters(preloadChooser);
    p3Chooser.updateFilters(p2Chooser);

    Logger.processInputs(prefix + "/SmartDashboard", inputs);

    preloadChooser.updateDisplay(autoDisplay, true);
    p2Chooser.updateDisplay(autoDisplay, false);
    p3Chooser.updateDisplay(autoDisplay, false);
  }
}
