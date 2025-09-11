package frc.robot.subsystems.leds;

import com.ctre.phoenix6.signals.StripTypeValue;
import frc.robot.util.CanID;

public class LedConstants {
  public static final CanID canId = new CanID(0);

  public static final StripTypeValue ledStripType = StripTypeValue.RGB;
  public static final double brightness = 0.3;

  public static final int numCanivoreInternalLeds = 8;

  public static final int rightStartIndex = 0;
  public static final int rightEndIndex = 22;

  public static final int centerStartIndex = 23;
  public static final int centerEndIndex = 34;

  public static final int leftStartIndex = 35;
  public static final int leftEndIndex = 53;

  public static final int endIndex = 53;
}
