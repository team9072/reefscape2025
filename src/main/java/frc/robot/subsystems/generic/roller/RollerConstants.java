package frc.robot.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import frc.robot.util.CanID;

public class RollerConstants {
  public Current baseCurrentLimit = Amps.of(40);
  public Current spikeCurrentLimit = Amps.of(40);
  public Time spikeTime = Seconds.of(1);

  public Current statorCurrentLimit = Amps.of(120);

  public InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;
  public NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public double motorReduction = 1.0;

  public final CanID canId;

  public RollerConstants(CanID canId) {
    this.canId = canId;
  }

  public RollerConstants withBaseCurrentLimit(Current currentLimit) {
    this.baseCurrentLimit = currentLimit;
    return this;
  }

  public RollerConstants withSpikeCurrentLimit(Current currentLimit, Time spikeTime) {
    this.spikeCurrentLimit = currentLimit;
    this.spikeTime = spikeTime;
    return this;
  }

  public RollerConstants withStatorCurrentLimit(Current currentLimit) {
    this.statorCurrentLimit = currentLimit;
    return this;
  }

  public RollerConstants withInvert(InvertedValue invertedValue) {
    this.invertedValue = invertedValue;
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
