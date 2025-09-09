package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.generic.roller.RollerConstants;
import frc.robot.util.CanID;

public class CoralPlacerConstants {
  public enum CoralPlacerPosition {
    stowPosition(Rotations.of(0.25)),
    grabPosition(Rotations.of(0.255)),

    holdPosition(Rotations.of(-0.23)),
    preScoreHoldPosition(Rotations.of(-0.1)),
    preScoreHoldPositionL4(Rotations.of(-0.1)),

    scorePosition(Rotations.of(0.14)),
    scoreTroughPosition(Rotations.of(0.1)),

    grabReefAlgaePosition(Rotations.of(0.0)),
    grabGroundAlgaePosition(Rotations.of(0.134)),
    grabGroundCoralPosition(Rotations.of(0.125)),
    algaeStowPosition(Rotations.of(-0.215)),

    scoreBargePosition(Rotations.of(-0.35)),
    scoreProcessorPosition(Rotations.of(0.03)),

    unstuckPosition(Rotations.of(-0.56));

    public static final AngularVelocity velocityTolerance = RotationsPerSecond.of(0.05);
    public static final Angle scoreCompleteThreshold = Rotations.of(0.13);
    public static final Angle jogAmmount = Rotations.of(0.025);

    public Angle angle;

    CoralPlacerPosition(Angle angle) {
      this.angle = angle;
    }
  }

  public static final Angle cupHitMaxAngle = Rotations.of(0.248);
  public static final Angle cupHitMinAngle = Rotations.of(0.14);

  public static final Angle topHitMaxAngle = Rotations.of(-0.26);
  public static final Angle topHitMinAngle = Rotations.of(-1);

  public static final Current currentLimit = Amps.of(40);
  public static final Current statorCurrentLimit = Amps.of(60);
  public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;

  public static final double rotorToSensor = (72.0 / 18.0) * 5.0;
  public static final double sensorToMechanism = (72.0 / 24.0);
  public static final double motorReduction = rotorToSensor * sensorToMechanism;

  public static final Angle maxAngle = Rotations.of(0.255);
  public static final Angle minAngle = Rotations.of(-0.4);

  public static final double kG = -0.07;
  public static final double kP = 100.0;
  public static final double kD = 0.0;
  public static final double kV = 10;

  public static final double rampVelocity = 2;
  public static final double rampVelocityRemoveAlgae = 1;
  public static final double rampAcceleration = 5;

  public static final Distance armLength = Inches.of(5);
  public static final Mass armMass = Pounds.of(1.3);
  public static final MomentOfInertia armMoi =
      KilogramSquareMeters.of(Math.pow(armLength.in(Meters) / 2, 2) * armMass.in(Kilograms));

  public static final Angle encoderOffsetStow = Rotations.of(0.0);
  public static final Angle encoderOffset =
      encoderOffsetStow.minus(CoralPlacerPosition.stowPosition.angle.div(sensorToMechanism));

  public static final String canBus = ""; // On rio bus
  public static final CanID pivotCanId = new CanID(15, canBus);
  public static final CanID pivotEncoderCanId = new CanID(5, canBus);
  public static final CanID rollerCanId = new CanID(16, canBus);

  public static final int beamBreakDioId = 9;

  public static final RollerConstants roller =
      new RollerConstants(rollerCanId)
          .withNeutralMode(NeutralModeValue.Coast)
          .withInvert(InvertedValue.CounterClockwise_Positive)
          .withStatorCurrentLimit(Amps.of(130))
          .withBaseCurrentLimit(Amps.of(30))
          .withSpikeCurrentLimit(Amps.of(40), Seconds.of(1));

  public static final Current rollerHoldAlgaeTorque = Amps.of(40);
  public static final double rollerHoldAlgaeDutyCycle = 0.25;

  public static final Current rollerHoldCoralTorque = Amps.of(35);
  public static final double rollerHoldCoralDutyCycle = 0.1;

  public static final Current rollerGrabAlgaeTorque = Amps.of(90);
  public static final double rollerGrabAlgaeDutyCycle = 0.6;

  public static final Current rollerReverseTorque = Amps.of(-100);
  public static final double rollerReverseDutyCycle = 0.2;
}
