package frc.robot.commands.autos.chooser;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.autos.AutoPathSegments.AutoPathSegment;
import frc.robot.commands.autos.AutoPositions.CoralSpike;
import java.util.ArrayList;
import java.util.List;

public class ManagedChooser<V extends Enum<V> & AutoPathSegment> {
  private static boolean flipTrajectoryDisplay() {
    return DriverStation.getAlliance().map((alliance) -> alliance == Alliance.Red).orElse(false);
  }

  public final String key;
  private final TrajectoryChooser<V> chooser;
  public V selectedTrajectory;

  private AutoPathSegment priorTrajectory;
  private ManagedChooser<?> priorChooser;

  public ManagedChooser(
      String key, V[] trajectories, ManagedChooser<?> priorChooser, V defaultTrajectory) {
    this.key = key;
    this.chooser = new TrajectoryChooser<V>(trajectories);
    this.priorChooser = priorChooser;

    chooser.select(defaultTrajectory);

    if (priorChooser != null) {
      updateFilters(true);
    }

    SmartDashboard.putData(key, chooser);
  }

  private List<CoralSpike> getUsedSpikes() {
    final List<CoralSpike> usedSpikes;

    if (priorChooser != null) {
      usedSpikes = priorChooser.getUsedSpikes();
    } else {
      usedSpikes = new ArrayList<>();
    }

    if (selectedTrajectory.spikeTaken().isPresent()) {
      usedSpikes.add(selectedTrajectory.spikeTaken().get());
    }

    return usedSpikes;
  }

  private List<CoralSpike> getPriorSpikes() {
    if (priorChooser != null) {
      return priorChooser.getUsedSpikes();
    } else {
      return List.of();
    }
  }

  public void updateSelected() {
    selectedTrajectory = chooser.getSelected();
  }

  public void updateFilters(boolean force) {
    if (priorChooser == null || !force && priorChooser.selectedTrajectory == priorTrajectory) {
      return;
    }

    this.priorTrajectory = priorChooser.selectedTrajectory;
    if (priorTrajectory == null) {
      chooser.filterOptions((trajectory) -> false);
    } else {
      chooser.filterOptions(
          (trajectory) -> {
            final boolean takesPriorSpike;

            if (trajectory.spikeTaken().isPresent()) {
              takesPriorSpike = getPriorSpikes().contains(trajectory.spikeTaken().get());
            } else {
              takesPriorSpike = false;
            }

            return trajectory.initialPosition() == priorTrajectory.finalPosition()
                && !takesPriorSpike;
          });
    }
  }

  public void updateFilters() {
    updateFilters(false);
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
