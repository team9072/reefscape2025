package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorCalculations.distanceToDrumRotation;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CanID;

public class ElevatorConstants {
  public enum ElevatorPosition {
    intakePosition(Rotations.of(0)),
    reefL2Position(Rotations.of(0)),
    reefL3Position(Rotations.of(3.7));

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

  public static final Current currentLimit = Amps.of(40);

  public static final Angle minDistance = Rotations.of(0);
  public static final Angle maxDistance = Rotations.of(5.6);

  public static final double kP = 100;
  public static final double kV = 7.7;
  public static final double kG = 0.15;

  public static final double rampVelocity = 1.8;
  public static final double rampAcceleration = 5;

  public static final Mass elevatorMass = Pounds.of(25);
  public static final Distance drumRadius = Inches.of(1.432 / 2);

  public static double motorReduction = Math.pow(4, 3);

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static final CanID motorCanId = new CanID(11, canBus);

  public static final Voltage homingVoltage = Volts.of(3);
}
