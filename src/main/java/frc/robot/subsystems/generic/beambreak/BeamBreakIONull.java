package frc.robot.subsystems.generic.beambreak;

public class BeamBreakIONull implements BeamBreakIO {
  @Override
  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.objectDetected = false;
  }
}
