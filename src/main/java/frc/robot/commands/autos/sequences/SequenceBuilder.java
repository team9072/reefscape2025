package frc.robot.commands.autos.sequences;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import choreo.util.ChoreoAlert;
import choreo.util.ChoreoAlert.MultiAlert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.autos.AutoPathSegments.AutoPathSegment;
import frc.robot.commands.autos.AutoPathSegments.IntakeTrajectory;
import frc.robot.commands.autos.AutoPathSegments.PreloadTrajectory;
import java.util.Collections;
import java.util.List;

public class SequenceBuilder {
  private final MultiAlert invalidPathType =
      ChoreoAlert.multiAlert((causes) -> "Invalid path type: " + causes, AlertType.kError);

  private final AutoSequences autoSequences;

  public SequenceBuilder(Robot.Subsystems s) {
    autoSequences = new AutoSequences(s);
  }

  public AutoRoutine buildRoutine(List<AutoPathSegment> pathSegments) {
    AutoRoutine routine = autoSequences.getFactory().newRoutine("Auto Routine");

    if (pathSegments.size() == 0) {
      return routine;
    }

    Collections.reverse(pathSegments);
    AutoTrajectory nextPath = null;
    for (AutoPathSegment path : pathSegments) {
      nextPath = handlePathSegment(path, routine, nextPath);
    }

    routine.active().onTrue(Commands.sequence(nextPath.resetOdometry(), nextPath.cmd()));

    return routine;
  }

  private AutoTrajectory handlePathSegment(
      AutoPathSegment path, AutoRoutine routine, AutoTrajectory nextPath) {
    if (path instanceof PreloadTrajectory) {
      return handlePreloadPath(path.trajectory(), routine, nextPath);
    } else if (path instanceof IntakeTrajectory) {
      return handleIntakePath(path.trajectory(), routine, nextPath);
    } else {
      invalidPathType.addCause(path.getClass().getName());
      return nextPath;
    }
  }

  private AutoTrajectory handlePreloadPath(
      Trajectory<SwerveSample> trajectory, AutoRoutine routine, AutoTrajectory nextPath) {
    AutoTrajectory preloadTrajectory = routine.trajectory(trajectory);

    autoSequences.scorePreload(preloadTrajectory, nextPath);

    return preloadTrajectory;
  }

  private AutoTrajectory handleIntakePath(
      Trajectory<SwerveSample> trajectory, AutoRoutine routine, AutoTrajectory nextPath) {
    AutoTrajectory preloadTrajectory = routine.trajectory(trajectory);

    // autoSequences.scorePreload(preloadTrajectory, nextPath);

    return preloadTrajectory;
  }
}
