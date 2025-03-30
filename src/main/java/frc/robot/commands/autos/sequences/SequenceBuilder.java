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
import frc.robot.commands.autos.AutoPositions.ReefPole;
import frc.robot.commands.scoring.ScoringPosition;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

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

    Map<ReefPole, ScoringPosition> poleScoringMap = new HashMap<>();
    List<ScoringPosition> scoringPositions = new ArrayList<>();
    for (AutoPathSegment path : pathSegments) {
      if (path.finalPosition() instanceof ReefPole finalPole) {
        ScoringPosition availablePosition =
            poleScoringMap.getOrDefault(finalPole, ScoringPosition.branchL4);
        poleScoringMap.put(finalPole, ScoringPosition.branchL2);

        scoringPositions.add(availablePosition);
      } else {
        scoringPositions.add(ScoringPosition.branchL4);
      }
    }

    Collections.reverse(pathSegments);
    AutoTrajectory nextPath = null;
    for (AutoPathSegment path : pathSegments) {
      ScoringPosition scoringPosition = scoringPositions.remove(scoringPositions.size() - 1);
      nextPath = handlePathSegment(scoringPosition, path, routine, nextPath);
    }

    routine.active().onTrue(Commands.sequence(nextPath.resetOdometry(), nextPath.cmd()));

    return routine;
  }

  private AutoTrajectory handlePathSegment(
      ScoringPosition scoringPosition,
      AutoPathSegment path,
      AutoRoutine routine,
      AutoTrajectory nextPath) {
    if (path instanceof PreloadTrajectory) {
      return handlePreloadPath(scoringPosition, path.trajectory(), routine, nextPath);
    } else if (path instanceof IntakeTrajectory) {
      return handleIntakePath(scoringPosition, path.trajectory(), routine, nextPath);
    } else {
      invalidPathType.addCause(path.getClass().getName());
      return nextPath;
    }
  }

  private AutoTrajectory handlePreloadPath(
      ScoringPosition scoringPosition,
      Trajectory<SwerveSample> trajectory,
      AutoRoutine routine,
      AutoTrajectory nextPath) {
    AutoTrajectory preloadTrajectory = routine.trajectory(trajectory);

    autoSequences.scorePreload(scoringPosition, preloadTrajectory, nextPath);

    return preloadTrajectory;
  }

  private AutoTrajectory handleIntakePath(
      ScoringPosition scoringPosition,
      Trajectory<SwerveSample> trajectory,
      AutoRoutine routine,
      AutoTrajectory nextPath) {
    AutoTrajectory intakeTrajectory = routine.trajectory(trajectory.getSplit(0).get());
    AutoTrajectory scoreTrajectory = routine.trajectory(trajectory.getSplit(1).get());

    autoSequences.intakeAndScore(scoringPosition, intakeTrajectory, scoreTrajectory, nextPath);

    return intakeTrajectory;
  }
}
