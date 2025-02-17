package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CanID;

public class CoralPlacerConstants {
  public static final Voltage intakeVoltage = Volts.of(1);

  public static final Voltage extendVoltage = Volts.of(3);
  public static final Voltage retractVoltage = Volts.of(-3);

  public static final Current currentLimit = Amps.of(40);
  public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
  public static final double motorReduction = (48.0 / 28.0) * (36.0 / 18.0);

  public static final double kG = 0;
  public static final double kP = 20;
  public static final double kD = 0;
  public static final double kV = 0.6;

  public static final double rampVelocity = 5.5;
  public static final double rampAcceleration = 15;

  public static final Distance armLength = Inches.of(5);
  public static final Mass armMass = Pounds.of(1.3);
  public static final MomentOfInertia armMoi =
      KilogramSquareMeters.of(Math.pow(armLength.in(Meters) / 2, 2) * armMass.in(Kilograms));

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static final CanID motorCanId = new CanID(17, canBus);

  public static final int beamBreakDioId = 0;
}
