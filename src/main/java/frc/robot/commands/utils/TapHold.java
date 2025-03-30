package frc.robot.commands.utils;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.BooleanSupplier;

public class TapHold {
  // Goes high for one cycle after tap
  public final Trigger tap;

  // Goes high for the duration of hold
  public final Trigger hold;

  public TapHold(BooleanSupplier conition, double debounceSeconds) {
    hold = new Trigger(conition).debounce(debounceSeconds);

    tap =
        new Trigger(
            new BooleanSupplier() {
              private final Timer timer = new Timer();
              private boolean previous = false;

              @Override
              public boolean getAsBoolean() {
                boolean current = conition.getAsBoolean();
                boolean res = false;

                if (current && !previous) {
                  // Risng edge
                  timer.restart();
                } else if (previous && !current) {
                  // Falling edge
                  res = !timer.hasElapsed(debounceSeconds);
                }

                previous = current;
                return res;
              }
            });
  }

  public TapHold(Trigger trigger, Time debounceTime) {
    this(trigger, debounceTime.in(Seconds));
  }
}
