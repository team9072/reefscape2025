package frc.robot.subsystems.coralplacer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.beambreak.BeamBreakIO;
import frc.robot.subsystems.generic.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class CoralPlacer extends SubsystemBase {
  private final RollerIO leftRollerIO;
  private final RollerIOInputsAutoLogged leftRollerInputs = new RollerIOInputsAutoLogged();

  private final RollerIO rightRollerIO;
  private final RollerIOInputsAutoLogged rightRollerInputs = new RollerIOInputsAutoLogged();

  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  public CoralPlacer(RollerIO leftRollerIO, RollerIO rightRollerIO, BeamBreakIO beamBreakIO) {
    this.leftRollerIO = leftRollerIO;
    this.rightRollerIO = rightRollerIO;
    this.beamBreakIO = beamBreakIO;
  }

  @Override
  public void periodic() {
    rightRollerIO.updateInputs(rightRollerInputs);
    Logger.processInputs("Coral Placer/Left Roller", rightRollerInputs);

    leftRollerIO.updateInputs(leftRollerInputs);
    Logger.processInputs("Coral Placer/Right Roller", rightRollerInputs);

    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("Coral Placer/Beam Break", beamBreakInputs);
  }
}
