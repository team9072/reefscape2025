package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Current;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.generic.roller.RollerConstants;
import frc.robot.util.CanID;

public class IntakeConstants {
  public static final String canBus = TunerConstants.kCANBus.getName();

  public static final CanID pivotCanId = new CanID(0, canBus); // TODO: Update id
  public static final CanID passthroughCanId = new CanID(11, canBus);
  public static final CanID rollerCanId = new CanID(12, canBus);

  public static final Current intakeTorqueCurrent = Amps.of(200);
  public static final double intakeTorqueDutyCycle = 0.5;

  public static PivotConstants pivot =
      new PivotConstants(pivotCanId, InvertedValue.Clockwise_Positive);
  public static RollerConstants roller =
      new RollerConstants(rollerCanId)
          .withBaseCurrentLimit(Amps.of(20))
          .withSpikeCurrentLimit(Amps.of(80), Seconds.of(0.2))
          .withStatorCurrentLimit(Amps.of(500));

  public static RollerConstants pasthrough =
      new RollerConstants(passthroughCanId)
          .withBaseCurrentLimit(Amps.of(20))
          .withSpikeCurrentLimit(Amps.of(80), Seconds.of(0.2))
          .withStatorCurrentLimit(Amps.of(500));
}
