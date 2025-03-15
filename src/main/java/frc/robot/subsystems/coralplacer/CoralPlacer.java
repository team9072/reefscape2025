package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import org.littletonrobotics.junction.Logger;

public class CoralPlacer extends SubsystemBase {
  private final CoralPlacerIO coralPlacerIO;
  private final CoralPlacerInputsAutoLogged coralPlacerInputs = new CoralPlacerInputsAutoLogged();

  public CoralPlacer(CoralPlacerIO coralPlacerIO) {
    this.coralPlacerIO = coralPlacerIO;
  }

  @Override
  public void periodic() {
    coralPlacerIO.updateInputs(coralPlacerInputs);
    Logger.processInputs("Coral Placer", coralPlacerInputs);
  }

  private boolean isAtPosition(CoralPlacerPosition position) {
    return position.angle.isNear(coralPlacerInputs.position, Degrees.of(2));
  }

  public Command setPosition(CoralPlacerPosition position) {
    return run(() -> coralPlacerIO.setPosition(position)).until(() -> isAtPosition(position));
  }
}
