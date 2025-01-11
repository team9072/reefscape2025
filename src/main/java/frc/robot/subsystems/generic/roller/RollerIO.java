package frc.robot.subsystems.generic.roller;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface RollerIO {
  @AutoLog
  public static class RollerIOInputs {
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
  }

  public default void updateInputs(RollerIOInputs inputs) {}

  public default void setVoltage(Voltage voltage) {}

  public default void stop() {
    setVoltage(Volts.zero());
  }
}
