package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import org.littletonrobotics.junction.AutoLog;

public interface CoralPlacerIO {
  @AutoLog
  public class CoralPlacerInputs {
    public Angle position = Radians.zero();
    public AngularVelocity velocity = RadiansPerSecond.zero();
    public Voltage appliedVoltage = Volts.zero();
    public Current current = Amps.zero();
  }

  public default void updateInputs(CoralPlacerInputs inputs) {}

  public default void setPosition(CoralPlacerPosition positon) {}

  /**
   * Since the positions are in a range of 0 - 1 rotations with continuous wrap off, we need to
   * reset into that range so it does not go backwards.
   */
  public default void normalizePosition() {}
}
