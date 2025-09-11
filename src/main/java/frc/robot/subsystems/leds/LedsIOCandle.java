package frc.robot.subsystems.leds;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.leds.LedAnimation.LedColor;

public class LedsIOCandle implements LedsIO {
  private final CANdle candle;

  private final StatusSignal<Voltage> supplyVolts;
  private final SolidColor colorRequest = new SolidColor(0, 0);

  public LedsIOCandle() {
    candle = LedConstants.canId.getCandle();
    supplyVolts = candle.getSupplyVoltage();

    var config = new CANdleConfiguration();
    config.LED.StripType = LedConstants.ledStripType;
    config.LED.BrightnessScalar = LedConstants.brightness;

    tryUntilOk(5, () -> candle.getConfigurator().apply(config, 0.25));
  }

  @Override
  public void updateInputs(LedsIOInputs inputs) {
    inputs.voltage = supplyVolts.getValue();
  }

  @Override
  public void setLeds(LedColor color, int startIdx, int endIndex) {
    candle.setControl(
        colorRequest
            .withColor(new RGBWColor(color.r, color.g, color.b, color.w))
            .withLEDStartIndex(startIdx)
            .withLEDEndIndex(endIndex));
  }
}
