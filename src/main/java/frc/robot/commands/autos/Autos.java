package frc.robot.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.commands.autos.AutoPathSegments.AutoPathSegment;
import frc.robot.commands.autos.chooser.AutoBuilder;
import frc.robot.commands.autos.sequences.SequenceBuilder;
import java.util.List;
import java.util.Optional;

public class Autos {
  private final AutoBuilder autoBuilder;
  private final SequenceBuilder sequenceBuilder;

  private Optional<Alliance> allianceAtGeneration = Optional.empty();
  private List<AutoPathSegment> pathsAtGeneration = List.of();
  private Command generatedCommand = Commands.none();

  public Autos(Robot.Subsystems s) {
    autoBuilder = new AutoBuilder("Auto Builder");
    sequenceBuilder = new SequenceBuilder(s);
  }

  public void update() {
    updateCache(autoBuilder.getTrajectories(), false);
  }

  public Command selectedAuto() {
    if (RobotBase.isSimulation() && pathsAtGeneration.equals(List.of())) {
      updateCache(autoBuilder.getTrajectories(), true);
    }

    return generatedCommand;
  }

  private void updateCache(List<AutoPathSegment> pathSegments, boolean force) {
    if (pathsAtGeneration.equals(pathSegments)
        && allianceAtGeneration.equals(DriverStation.getAlliance())) {
      return;
    }

    boolean dsValid = DriverStation.isDisabled() && DriverStation.getAlliance().isPresent();
    if (dsValid || force) {
      allianceAtGeneration = DriverStation.getAlliance();
      pathsAtGeneration = pathSegments;
      generatedCommand = sequenceBuilder.buildRoutine(pathSegments).cmd();
    } else {
      allianceAtGeneration = Optional.empty();
      pathsAtGeneration = List.of();
      generatedCommand = Commands.none();
    }
  }
}
