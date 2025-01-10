package frc.robot.subsystems.coralplacer;

import frc.robot.subsystems.generic.roller.RollerConstants;
import frc.robot.util.CanID;

public class CoralPlacerConstants {
  public static final String canBus = "";

  public static final int beamBreakDioId = 0;

  public static final CanID leftCanId = new CanID(0, canBus);
  public static final CanID rightCanId = new CanID(0, canBus);

  public static final RollerConstants leftRoller = new RollerConstants(leftCanId);
  public static final RollerConstants rightRoller = new RollerConstants(rightCanId);
}
