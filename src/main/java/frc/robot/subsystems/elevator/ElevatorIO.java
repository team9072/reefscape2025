package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public class ElevatorIOInputs {
    public Distance position = Meters.zero();
    public LinearVelocity velocity = MetersPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setHomePosition() {}
}
