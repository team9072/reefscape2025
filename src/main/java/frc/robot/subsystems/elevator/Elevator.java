package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO elevatorIO;
  private final ElevatorIOInputsAutoLogged elevatorInputs = new ElevatorIOInputsAutoLogged();
  private Angle desiredRotation = Degrees.of(0.0);

  public Elevator(ElevatorIO elevatorIO) {
    this.elevatorIO = elevatorIO;
  }

  @Override
  public void periodic() {
    elevatorIO.updateInputs(elevatorInputs);
    Logger.processInputs("Elevator", elevatorInputs);
  }

  private void updateDesiredRotation(Angle rotation) {
    desiredRotation = rotation;

    if (desiredRotation.gt(ElevatorConstants.maxDistance)) {
      desiredRotation = ElevatorConstants.maxDistance;
    }

    if (desiredRotation.lt(ElevatorConstants.minDistance)) {
      desiredRotation = ElevatorConstants.minDistance;
    }
  }

  public boolean atPosition(ElevatorPosition position) {
    return position.withinTolerance(elevatorInputs.rotation);
  }

  public Command setPosition(ElevatorPosition position) {
    return Commands.sequence(
        startRun(
            () -> elevatorIO.setPosition(position.angle),
            () -> updateDesiredRotation(elevatorInputs.rotation)),
        Commands.waitUntil(() -> atPosition(position)));
  }

  public Command moveJoystick(DoubleSupplier movementSupplier) {
    return run(
        () -> {
          updateDesiredRotation(desiredRotation.plus(Degrees.of(movementSupplier.getAsDouble())));
          elevatorIO.setPosition(desiredRotation);
        });
  }
}
