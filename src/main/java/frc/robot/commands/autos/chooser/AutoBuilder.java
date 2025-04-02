package frc.robot.commands.autos.chooser;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autos.AutoPathSegments.AutoPathSegment;
import frc.robot.commands.autos.AutoPathSegments.IntakeTrajectory;
import frc.robot.commands.autos.AutoPathSegments.PreloadTrajectory;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.littletonrobotics.junction.networktables.LoggedNetworkInput;

public class AutoBuilder extends LoggedNetworkInput {
  private final String key;
  private final Field2d autoDisplay = new Field2d();

  private final List<ManagedChooser<?>> choosers;

  private final LoggableInputs inputs =
      new LoggableInputs() {
        private <E extends Enum<E> & AutoPathSegment> void addChooserToLog(
            ManagedChooser<E> chooser, LogTable table) {
          table.put(chooser.key, chooser.selectedTrajectory);
        }

        public void toLog(LogTable table) {
          for (ManagedChooser<?> chooser : choosers) {
            addChooserToLog(chooser, table);
          }
        }

        private <E extends Enum<E> & AutoPathSegment> void setChooserFromLog(
            ManagedChooser<E> chooser, LogTable table) {
          chooser.selectedTrajectory = table.get(chooser.key, chooser.selectedTrajectory);
        }

        public void fromLog(LogTable table) {
          for (ManagedChooser<?> chooser : choosers) {
            setChooserFromLog(chooser, table);
          }
        }
      };

  public AutoBuilder(String key) {
    this.key = key;
    ManagedChooser<PreloadTrajectory> preloadChooser =
        new ManagedChooser<>(
            getKey("Preload"), PreloadTrajectory.values(), null, PreloadTrajectory.C1S_B);
    ManagedChooser<IntakeTrajectory> p2Chooser =
        new ManagedChooser<>(
            getKey("Second Piece"),
            IntakeTrajectory.values(),
            preloadChooser,
            IntakeTrajectory.B_S2_A);
    ManagedChooser<IntakeTrajectory> p3Chooser =
        new ManagedChooser<>(
            getKey("Third Piece"), IntakeTrajectory.values(), p2Chooser, IntakeTrajectory.A_S1_B);

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

    for (ManagedChooser<?> chooser : choosers) {
      chooser.updateFilters();
    }

    Logger.processInputs(prefix + "/SmartDashboard", inputs);

    boolean firstChooser = true;
    for (ManagedChooser<?> chooser : choosers) {
      chooser.updateDisplay(autoDisplay, firstChooser);
      firstChooser = false;
    }
  }

  public List<AutoPathSegment> getTrajectories() {
    final List<AutoPathSegment> trajectories = new ArrayList<>();

    for (ManagedChooser<?> chooser : choosers) {
      AutoPathSegment nullableTrajectory = chooser.selectedTrajectory;

      if (nullableTrajectory == null) {
        break;
      }

      trajectories.add(nullableTrajectory);
    }

    return trajectories;
  }
}
