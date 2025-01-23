package frc.robot.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.util.CanID;

public class RollerConstants {
  public Current currentLimit = Amps.of(40);
  public NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public double motorReduction = 1.0;

  public final CanID canId;

  public RollerConstants(CanID canId) {
    this.canId = canId;
  }

  public RollerConstants withCurrentLimits(Current currentLimit) {
    this.currentLimit = currentLimit;
    return this;
  }

  public RollerConstants withNeutralMode(NeutralModeValue neutralMode) {
    this.neutralMode = neutralMode;
    return this;
  }

  public RollerConstants withMotorReduction(double motorReduction) {
    this.motorReduction = motorReduction;
    return this;
  }
}
