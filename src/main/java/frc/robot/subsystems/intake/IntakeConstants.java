package frc.robot.subsystems.intake;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.generic.roller.RollerConstants;
import frc.robot.util.CanID;

public class IntakeConstants {
  public static final String canBus = TunerConstants.kCANBus.getName();

  public static final CanID pivotCanId = new CanID(11, canBus);
  public static final CanID rollerCanId = new CanID(12, canBus);

  public static PivotConstants pivot =
      new PivotConstants(pivotCanId, InvertedValue.Clockwise_Positive);
  public static RollerConstants roller = new RollerConstants(rollerCanId);
}
