package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.util.CanID;

public class PivotConstants {
  public static final Current currentLimit = Amps.of(40);
  public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
  public static final double motorReduction = 9.0 * (46.0 / 26.0);

  public static final Angle stowAngle = Rotations.of(-0.25);
  public static final Angle deployAngle = Rotations.of(0.07);

  public static final Distance armLength = Inches.of(14);
  public static final Mass armMass = Pounds.of(14);
  public static final MomentOfInertia armMoi =
      KilogramSquareMeters.of(Math.pow(armLength.in(Meters) / 2, 2) * armMass.in(Kilograms));

  public static final double kG = -0.45;
  public static final double kP = 40;
  public static final double kD = 4;
  public static final double kV = 2.4;

  public static final double rampVelocity = 3;
  public static final double rampAcceleration = 3;

  public final CanID canId;
  public final InvertedValue invertedValue;

  public PivotConstants(CanID canId, InvertedValue invertedValue) {
    this.canId = canId;
    this.invertedValue = invertedValue;
  }
}
