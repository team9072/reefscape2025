package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.CanID;

public class ElevatorConstants {
  public static Current currentLimit = Amps.of(40);

  public static Angle minDistance = Degrees.zero();
  public static Angle maxDistance = Degrees.of(0);

  public static final Mass elevatorMass = Kilograms.zero();
  public static final Distance drumRadius = Meters.zero();

  public static double motorReduction = 1.0;

  public static final String canBus = "";
  public static CanID motorCanId = new CanID(0, canBus);

  public static Voltage homingVoltage = Volts.of(3);
}
