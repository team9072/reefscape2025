package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.generic.roller.RollerConstants;
import frc.robot.util.CanID;

public class CoralPlacerConstants {
  public static final Voltage intakeVoltage = Volts.of(1);

  public static final Voltage extendVoltage = Volts.of(3);
  public static final Voltage retractVoltage = Volts.of(-3);

  public static final String canBus = TunerConstants.kCANBus.getName();

  public static final int beamBreakDioId = 0;

  public static final CanID leftCanId = new CanID(10, canBus);
  public static final CanID rightCanId = new CanID(9, canBus);

  public static final RollerConstants leftRoller = new RollerConstants(leftCanId);
  public static final RollerConstants rightRoller = new RollerConstants(rightCanId);
}
