package frc.robot.subsystems.coralplacer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.generic.beambreak.BeamBreakIO;
import frc.robot.subsystems.generic.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralPlacerRollers extends SubsystemBase {
  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public final Trigger hasCoral = new Trigger(() -> beamBreakInputs.objectDetected);

  public CoralPlacerRollers(RollerIO rollerIO, BeamBreakIO beamBreakIO) {
    this.rollerIO = rollerIO;
    this.beamBreakIO = beamBreakIO;
  }

  @Override
  public void periodic() {
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Coral Placer/Rollers", rollerInputs);

    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("Coral Placer/Beam Break", beamBreakInputs);
  }

  public Command grab() {
    return runEnd(
        () ->
            rollerIO.setTorque(
                CoralPlacerConstants.rollerGrabTorque, CoralPlacerConstants.rollerGrabDutyCycle),
        () -> rollerIO.stop());
  }

  public Command reverse() {
    return runEnd(
        () ->
            rollerIO.setTorque(
                CoralPlacerConstants.rollerReverseTorque,
                CoralPlacerConstants.rollerReverseDutyCycle),
        () -> rollerIO.stop());
  }
}
