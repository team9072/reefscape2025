package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class ElevatorCalculations {
  public static Distance drumRotationToDistance(Angle theta, Distance drumRadius) {
    // Arc length formula: L = r * theta (radians)
    return Meters.of(drumRadius.in(Meters) * theta.in(Radians));
  }

  public static LinearVelocity drumVelocityToLinearVelocity(
      AngularVelocity thetaPerSecond, Distance drumRadius) {
    // Angular to linear velocity formula: L = r * thetaPerSecond (radians per second)
    return MetersPerSecond.of(drumRadius.in(Meters) * thetaPerSecond.in(RadiansPerSecond));
  }

  public static Angle distanceToDrumRotation(Distance distance, Distance drumRadius) {
    // Arc length formula: theta (radians) = L / r
    return Radians.of(distance.in(Meters) / drumRadius.in(Meters));
  }

  public static AngularVelocity linearVelocityToDrumVelocity(
      LinearVelocity distancePerSecond, Distance drumRadius) {
    // Linear to angular velocity formula:
    // thetaPerSecond (radians per second) = L / r
    return RadiansPerSecond.of(distancePerSecond.in(MetersPerSecond) / drumRadius.in(Meters));
  }
}
