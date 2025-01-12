package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

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
    reefL2Position(Rotations.of(2)),
    reefL3Position(Rotations.of(4));

    public Angle angle;

    ElevatorPosition(Angle angle) {
      this.angle = angle;
    }
  }

  public static Current currentLimit = Amps.of(40);

  public static Angle minDistance = Rotations.of(0);
  public static Angle maxDistance = Rotations.of(5.6);

  public static double kP = 100;

  public static final Mass elevatorMass = Kilograms.zero();
  public static final Distance drumRadius = Inches.of(1.432 / 2);

  public static double motorReduction = Math.pow(4, 3);

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static CanID motorCanId = new CanID(11, canBus);

  public static Voltage homingVoltage = Volts.of(3);
}
