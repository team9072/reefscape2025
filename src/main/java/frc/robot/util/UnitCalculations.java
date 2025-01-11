package frc.robot.util;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class UnitCalculations {
  public static Distance drumRotationToDistance(Angle theta, Distance drumRadius) {
    // Arc length formula: L = r * theta (radians)
    return drumRadius.times(theta.in(Radians));
  }

  public static LinearVelocity drumVelocityToLinearVelocity(
      AngularVelocity thetaPerSecond, Distance drumRadius) {
    // Angular to linear velocity formula: L = r * thetaPerSecond (radians per second)
    return drumRadius.times(thetaPerSecond.in(RadiansPerSecond)).per(Second);
  }
}
