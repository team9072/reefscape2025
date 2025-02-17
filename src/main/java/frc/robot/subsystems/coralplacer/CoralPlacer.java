package frc.robot.subsystems.coralplacer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
}
