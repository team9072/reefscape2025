package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralplacer.CoralPlacer;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;

public class CoralFlow {
  // TODO: account for pre-load
  private boolean hasCoral = false;

  private final Intake intake;
  private final Elevator elevator;
  private final CoralPlacer coralPlacer;

  public CoralFlow(Intake intake, Elevator elevator, CoralPlacer coralPlacer) {
    this.intake = intake;
    this.elevator = elevator;
    this.coralPlacer = coralPlacer;
  }

  public Command grabCoral() {
    return Commands.sequence(
        Commands.parallel(
            coralPlacer.setPosition(CoralPlacerPosition.grabPosition),
            elevator.setPosition(ElevatorPosition.readyPosition).withTimeout(0)),
        elevator.setPosition(ElevatorPosition.intakePosition),
        Commands.waitSeconds(0.2),
        elevator.setPosition(ElevatorPosition.readyPosition).until(elevator.clearsCoral),
        coralPlacer.setPosition(CoralPlacerPosition.readyPosition));
  }
}
