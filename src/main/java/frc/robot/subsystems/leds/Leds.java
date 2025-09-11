package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.LedAnimation.LedAnmationStep;
import frc.robot.subsystems.leds.LedAnimation.LedColor;
import org.littletonrobotics.junction.Logger;

public class Leds extends SubsystemBase {
  private final LedsIO ledsIO;
  private final LedsIOInputsAutoLogged ledsInputs = new LedsIOInputsAutoLogged();

  private LedAnimation animation = null;
  private int animationIndex = 0;
  private double animationDelay = 0.0;
  private final Timer animationTimer = new Timer();

  public Leds(LedsIO ledsIO) {
    this.ledsIO = ledsIO;
    animationTimer.start();
  }

  private void updateLedsFromAnimation(boolean restart) {
    if (animation == null || !(restart || animationTimer.hasElapsed(animationDelay))) {
      return;
    }

    LedAnmationStep animationStep = animation.getStep(animationIndex);
    do {
      ledsIO.setLeds(animationStep.color(), animationStep.startIdx(), animationStep.endIdx());

      animationIndex += 1;

      if (animationIndex >= animation.size()) {
        animationIndex = 0;
        break;
      }

      animationStep = animation.getStep(animationIndex);
    } while (animationStep.durationSecs() == 0);

    if (animation.isStatic()) {
      animationDelay = 10000;
    } else {
      animationDelay = animationStep.durationSecs();
    }

    animationTimer.restart();
  }

  @Override
  public void periodic() {
    ledsIO.updateInputs(ledsInputs);
    Logger.processInputs("Leds", ledsInputs);

    updateLedsFromAnimation(false);
  }

  public void setAnimation(LedAnimation anmation) {
    this.animation = anmation;
    this.animationIndex = 0;

    updateLedsFromAnimation(true);
  }

  public void clearAnimation() {
    this.animation = null;
    ledsIO.setAllLeds(LedColor.off);
  }
}
