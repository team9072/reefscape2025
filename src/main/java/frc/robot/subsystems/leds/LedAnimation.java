package frc.robot.subsystems.leds;

import java.util.ArrayList;
import java.util.List;

public class LedAnimation {
  public enum LedColor {
    off(0, 0, 0),
    red(255, 0, 0),
    green(0, 255, 0),
    blue(0, 0, 255),
    cyan(0, 255, 255),
    yellow(255, 255, 0),
    magenta(255, 0, 255),
    white(255, 255, 255);

    public final int r;
    public final int g;
    public final int b;
    public final int w;

    private LedColor(int r, int g, int b, int w) {
      this.r = r;
      this.g = g;
      this.b = b;
      this.w = w;
    }

    private LedColor(int r, int g, int b) {
      this(r, g, b, 0);
    }
  }

  public static record LedAnmationStep(
      double durationSecs, LedColor color, int startIdx, int endIdx) {}

  public static class Builder {
    private List<LedAnmationStep> animationSteps = new ArrayList<LedAnmationStep>();

    public LedAnimation.Builder setAllLeds(double duration, LedColor color) {
      return setLeds(duration, color, 0, LedConstants.endIndex);
    }

    public LedAnimation.Builder setLeds(double duration, LedColor color, int startIdx, int endIdx) {
      animationSteps.add(new LedAnmationStep(duration, color, startIdx, endIdx));
      return this;
    }

    public LedAnimation build() {
      if (animationSteps.size() == 0) {
        throw new IllegalStateException(
            "Called LedAnimation.Builder.build() with no animation steps");
      }

      boolean isStatic = true;
      for (LedAnmationStep step : animationSteps) {
        if (step.durationSecs() > 0) {
          isStatic = false;
          break;
        }
      }

      return new LedAnimation(List.copyOf(animationSteps), isStatic);
    }
  }

  private List<LedAnmationStep> animationSteps;
  private boolean isStatic;

  private LedAnimation(List<LedAnmationStep> animationSteps, boolean isStatic) {
    this.animationSteps = animationSteps;
    this.isStatic = isStatic;
  }

  public LedAnmationStep getStep(int index) {
    return animationSteps.get(index);
  }

  public int size() {
    return animationSteps.size();
  }

  public boolean isStatic() {
    return isStatic;
  }
}
