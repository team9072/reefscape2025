package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorCalculations.distanceToDrumRotation;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CanID;

public class ElevatorConstants {
  /**
   * Positions based on the absolute elevator zero position. The `startingPosition` variable in the
   * constructor determines how far up the elevator starts, which is subtracted from each position.
   */
  public enum ElevatorPosition {
    groundPickupPosition(Rotations.of(-0.55)),

    grabPosition(Rotations.of(-0.55)),
    readyPosition(Rotations.of(1.1)),

    reefTroughPosition(Rotations.of(1.2)),
    reefL2Position(Rotations.of(1.65)),
    reefL3Position(Rotations.of(5.05)),
    reefL4Position(Rotations.of(10.3)),

    algaeL2Position(Rotations.of(2.35)),
    algaeL3Position(Rotations.of(5.5)),

    clearBargePosition(Rotations.of(4.95));

    public static final Angle grabZoneMax = Rotations.of(1.75);

    public static final Distance linearTolerance = Inches.of(1);
    public static final Angle tolerance = distanceToDrumRotation(linearTolerance, drumRadius);

    public Angle angle;

    ElevatorPosition(Angle angle) {
      this.angle = angle;
    }

    public boolean withinTolerance(Angle angle) {
      return this.angle.isNear(angle, tolerance);
    }
  }

  public static final Current currentLimit = Amps.of(70);
  public static final Current statorLimit = Amps.of(120);

  public static final Angle minDistance = Rotations.of(-0.55);
  public static final Angle maxDistance = Rotations.of(10.5);

  public static final double kP = 10;
  public static final double kV = 0;
  public static final double kG = 0.47;

  public static final double rampVelocity = 30;
  public static final double rampAcceleration = 100;

  public static final Mass elevatorMass = Pounds.of(25);
  public static final Distance drumRadius = Inches.of(1.5 / 2);

  public static double motorReduction = 72.0 / 30.0;
  public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static final CanID primaryMotorCanId = new CanID(13, canBus);
  public static final CanID secondaryMotorCanId = new CanID(14, canBus);

  public static final Voltage homingVoltage = Volts.of(3);
}
