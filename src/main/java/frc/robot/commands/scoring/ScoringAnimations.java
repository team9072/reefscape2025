package frc.robot.commands.scoring;

import frc.robot.subsystems.leds.LedAnimation;
import frc.robot.subsystems.leds.LedAnimation.LedColor;

public class ScoringAnimations {
  public static final LedAnimation troughL1 =
      new LedAnimation.Builder().setAllLeds(0, LedColor.green).build();
  public static final LedAnimation branchL2 =
      new LedAnimation.Builder().setAllLeds(0, LedColor.blue).build();
  public static final LedAnimation branchL3 =
      new LedAnimation.Builder().setAllLeds(0, LedColor.red).build();
  public static final LedAnimation branchL4 =
      new LedAnimation.Builder().setAllLeds(0, LedColor.yellow).build();

  public static final LedAnimation algae =
      new LedAnimation.Builder().setAllLeds(0, LedColor.magenta).build();
}
