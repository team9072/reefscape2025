package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.leds.LedAnimation.LedColor;
import org.littletonrobotics.junction.AutoLog;

public interface LedsIO {
  @AutoLog
  public static class LedsIOInputs {
    public Voltage voltage = Volts.zero();
  }

  public default void updateInputs(LedsIOInputs inputs) {}

  public default void setAllLeds(LedColor color) {
    setLeds(color, 0, LedConstants.endIndex);
  }

  public default void setLeds(LedColor color, int startIdx, int endIndex) {}
}
