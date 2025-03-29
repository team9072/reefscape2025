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
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class AutoBuilder extends LoggedNetworkInput {
  private static class ManagedChooser<V extends Enum<V> & AutoTrajectory> {
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

  private final List<ManagedChooser<?>> choosers;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        private <E extends Enum<E> & AutoTrajectory> void addChooserToLog(
            ManagedChooser<E> chooser, LogTable table) {
          table.put(chooser.key, chooser.selectedTrajectory);
        }

        public void toLog(LogTable table) {
          for (ManagedChooser<?> chooser : choosers) {
            addChooserToLog(chooser, table);
          }
        }

        private <E extends Enum<E> & AutoTrajectory> void setChooserFromLog(
            ManagedChooser<E> chooser, LogTable table) {
          chooser.selectedTrajectory = table.get(chooser.key, chooser.selectedTrajectory);
        }

        public void fromLog(LogTable table) {
          for (ManagedChooser<?> chooser : choosers) {
            setChooserFromLog(chooser, table);
          }
        }
      };

  private static boolean flipTrajectoryDisplay() {
    return DriverStation.getAlliance().map((alliance) -> alliance == Alliance.Red).orElse(false);
  }

  public AutoBuilder(String key) {
    this.key = key;
    ManagedChooser<PreloadTrajectory> preloadChooser =
        new ManagedChooser<>(getKey("Preload"), PreloadTrajectory.values(), null);
    ManagedChooser<IntakeTrajectory> p2Chooser =
        new ManagedChooser<>(getKey("Second Piece"), IntakeTrajectory.values(), preloadChooser);
    ManagedChooser<IntakeTrajectory> p3Chooser =
        new ManagedChooser<>(getKey("Third Piece"), IntakeTrajectory.values(), p2Chooser);

    choosers = List.of(preloadChooser, p2Chooser, p3Chooser);

    SmartDashboard.putData(getKey("Auto Path"), autoDisplay);
    periodic();
    Logger.registerDashboardInput(this);
  }

  private String getKey(String path) {
    return key + "/" + path;
  }

  public void periodic() {
    if (!Logger.hasReplaySource()) {
      for (ManagedChooser<?> chooser : choosers) {
        chooser.updateSelected();
      }
    }

    ManagedChooser<?> priorChooser = null;
    for (ManagedChooser<?> chooser : choosers) {
      if (priorChooser != null) {
        chooser.updateFilters(priorChooser);
      }

      priorChooser = chooser;
    }

    Logger.processInputs(prefix + "/SmartDashboard", inputs);

    boolean firstChooser = true;
    for (ManagedChooser<?> chooser : choosers) {
      chooser.updateDisplay(autoDisplay, firstChooser);
      firstChooser = false;
    }
  }

  public List<AutoTrajectory> getTrajectories() {
    final List<AutoTrajectory> trajectories = List.of();

    for (ManagedChooser<?> chooser : choosers) {
      AutoTrajectory nullableTrajectory = chooser.selectedTrajectory;

      if (nullableTrajectory == null) {
        break;
      }

      trajectories.add(nullableTrajectory);
    }

    return trajectories;
  }
}
