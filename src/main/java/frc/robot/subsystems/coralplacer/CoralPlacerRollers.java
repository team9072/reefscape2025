package frc.robot.subsystems.coralplacer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralPlacerRollers extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  public CoralPlacerRollers(RollerIO rollerIO) {
    this.rollerIO = rollerIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Coral Placer/Rollers", rollerInputs);
  }

  public Command grab() {
    return runEnd(
        () ->
            rollerIO.setTorque(
                CoralPlacerConstants.rollerGrabTorque, CoralPlacerConstants.rollerGrabDutyCycle),
        () -> rollerIO.stop());
  }
}
