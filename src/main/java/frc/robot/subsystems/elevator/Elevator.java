package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();

  public final Trigger clearsCoral =
      new Trigger(
          () ->
              ElevatorPosition.readyPosition.withinTolerance(elevatorInputs.rotation)
                  || ElevatorPosition.readyPosition.angle.lt(elevatorInputs.rotation));

  public final Trigger finishedGrab =
      new Trigger(() -> elevatorInputs.rotation.lt(ElevatorPosition.grabZoneMax));

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
  }

  private boolean atPosition(ElevatorPosition position) {
    return position.withinTolerance(elevatorInputs.rotation);
  }

  public Command setPosition(ElevatorPosition position) {

    return Commands.sequence(
        runOnce(() -> elevatorIO.setPosition(position.angle)),
        Commands.waitUntil(
            position == ElevatorPosition.grabPosition ? finishedGrab : () -> atPosition(position)));
  }

  public Command clearCoral() {
    return setPosition(ElevatorPosition.readyPosition).until(clearsCoral).unless(clearsCoral);
  }
}
