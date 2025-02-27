package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

  private Command setPositionModulo(CoralPlacerPosition position) {
    return this.run(coralPlacerIO::normalizePosition)
        .until(() -> coralPlacerInputs.position.in(Rotations) < 1.0)
        .andThen(() -> coralPlacerIO.setPosition(position));
  }

  private boolean isAtPosition(CoralPlacerPosition position) {
    return position.angle.isNear(coralPlacerInputs.position, Degrees.of(2));
  }

  public Command setPosition(CoralPlacerPosition position) {
    return setPositionModulo(position).alongWith(Commands.waitUntil(() -> isAtPosition(position)));
  }
}
