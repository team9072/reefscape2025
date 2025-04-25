package frc.robot.subsystems.leds;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  private static final Animation demoModeAnimation = new RainbowAnimation();

  static {
    demoModeAnimation.setSpeed(0.25);
  }

  private final CANdle candle;

  public Leds() {
    candle = new CANdle(0);
  }

  public Command setColor(int r, int g, int b) {
    return runOnce(
            () -> {
              candle.clearAnimation(0);
              candle.setLEDs(r, g, b);
            })
        .ignoringDisable(true);
  }

  public Command off() {
    return setColor(0, 0, 0);
  }

  public Command animate(Animation animation) {
    return runOnce(() -> candle.animate(animation)).ignoringDisable(true);
  }

  public Command demoAnimation() {
    return animate(demoModeAnimation);
  }

  public Command driverAnimation() {
    return Commands.sequence(
            setColor(255, 0, 0), Commands.waitSeconds(1), off(), Commands.waitSeconds(1))
        .repeatedly();
  }
}
