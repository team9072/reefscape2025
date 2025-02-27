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
  public enum ElevatorPosition {
    intakePosition(Rotations.of(-0.1)),
    readyPosition(Rotations.of(1.5)),
    reefL2Position(Rotations.of(3.7)),
    reefL3Position(Rotations.of(6.47)),
    reefL4Position(Rotations.of(11.4));

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

  public static final Angle minDistance = Rotations.of(-0.4);
  public static final Angle maxDistance = Rotations.of(11.4);

  public static final double kP = 10;
  public static final double kV = 0;
  public static final double kG = 0.47;

  public static final double rampVelocity = 35 / 2;
  public static final double rampAcceleration = 50;

  public static final Mass elevatorMass = Pounds.of(25);
  public static final Distance drumRadius = Inches.of(1.5 / 2);

  public static double motorReduction = 72.0 / 30.0;
  public static final InvertedValue inverted = InvertedValue.Clockwise_Positive;

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static final CanID primaryMotorCanId = new CanID(13, canBus);
  public static final CanID secondaryMotorCanId = new CanID(14, canBus);

  public static final Voltage homingVoltage = Volts.of(3);
}
