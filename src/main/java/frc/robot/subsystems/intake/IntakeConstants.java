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

  public static final CanID pivotCanId = new CanID(19, canBus);
  public static final CanID rollerCanId = new CanID(18); // Intake rollers are on rio can bus
  public static final CanID passthroughCanId = new CanID(17, canBus);
  public static final CanID stagingRollerCanId = new CanID(16, canBus);

  public static final int beamBreakDioId = 1;

  public static final Current intakeTorqueCurrent = Amps.of(200);
  public static final Current intakeReverseTorqueCurrent = Amps.of(-100);
  public static final double intakeTorqueDutyCycle = 0.7;

  public static final Current intakeAlgaeTorqueCurrent = Amps.of(-30);
  public static final double intakeAlgaeTorqueDutyCycle = 0.2;

  public static final Current passthroughTorqueCurrent = Amps.of(200);
  public static final Current passthroughReverseTorqueCurrent = Amps.of(-100);
  public static final double passthroughTorqueDutyCycle = 0.55;

  public static final Current stagingTorqueCurrent = Amps.of(50);
  public static final Current stagingReverseTorqueCurrent = Amps.of(-50);
  public static final double stagingTorqueDutyCycle = 0.15;
  public static final Current stagingIdleTorqueCurrent = Amps.of(20);
  public static final double stagingIdleDutyCycle = 0.1;

  public static PivotConstants pivot =
      new PivotConstants(pivotCanId, InvertedValue.Clockwise_Positive);
  public static RollerConstants roller =
      new RollerConstants(rollerCanId)
          .withInvert(InvertedValue.Clockwise_Positive)
          .withBaseCurrentLimit(Amps.of(20))
          .withSpikeCurrentLimit(Amps.of(80), Seconds.of(0.2))
          .withStatorCurrentLimit(Amps.of(500));

  public static RollerConstants pasthrough =
      new RollerConstants(passthroughCanId)
          .withInvert(InvertedValue.Clockwise_Positive)
          .withBaseCurrentLimit(Amps.of(20))
          .withSpikeCurrentLimit(Amps.of(80), Seconds.of(0.2))
          .withStatorCurrentLimit(Amps.of(500));

  public static RollerConstants stagingRoller =
      new RollerConstants(stagingRollerCanId)
          .withInvert(InvertedValue.Clockwise_Positive)
          .withBaseCurrentLimit(Amps.of(20))
          .withSpikeCurrentLimit(Amps.of(80), Seconds.of(0.2))
          .withStatorCurrentLimit(Amps.of(500));
}
