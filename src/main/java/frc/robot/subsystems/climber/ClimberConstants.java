package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import frc.robot.generated.TunerConstants;
import frc.robot.util.CanID;

public class ClimberConstants {
  public enum ClimberPosition {
    stowPosition(Rotations.of(-0.5)),
    deployPosition(Rotations.of(-0.25)),
    climbPosition(Rotations.of(-0.5));

    public final Angle angle;

    ClimberPosition(Angle angle) {
      this.angle = angle;
    }
  }

  public static final double kP = 0.0;
  public static final double kD = 0.0;
  public static final double kG = 0.0;
  public static final double kV = 0.0;

  public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
  public static final InvertedValue invertedValue = InvertedValue.CounterClockwise_Positive;

  public static final double rampVelocity = 1;
  public static final double rampAcceleration = 1;

  public static final double gearboxReduction = 4 * 4 * 9;
  public static final double ropeReduction = 6.7;
  public static final double motorReduction = gearboxReduction * ropeReduction;

  public static final String canBus = TunerConstants.kCANBus.getName();
  public static final CanID canId = new CanID(21, canBus);
}
