package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.elevator.ElevatorCalculations.drumRotationToDistance;
import static frc.robot.subsystems.elevator.ElevatorCalculations.drumVelocityToLinearVelocity;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX motor;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(Rotations.of(0));

  public ElevatorIOTalonFX() {
    motor = ElevatorConstants.motorCanId.getTalon();
    positionRot = motor.getPosition();
    velocityRotPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.CurrentLimits.withSupplyCurrentLimit(ElevatorConstants.currentLimit);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.motorReduction;
    // Invert the motor
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    config.withSlot0(
        new Slot0Configs()
            .withKP(ElevatorConstants.kP)
            .withKV(ElevatorConstants.kV)
            .withKG(ElevatorConstants.kG));

    config.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ElevatorConstants.rampVelocity)
            .withMotionMagicAcceleration(ElevatorConstants.rampAcceleration));

    config.withSoftwareLimitSwitch(
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(ElevatorConstants.maxDistance)
            .withReverseSoftLimitThreshold(ElevatorConstants.minDistance)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true));

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.rotation = positionRot.getValue();
    inputs.angularVelocity = velocityRotPerSec.getValue();

    inputs.position = drumRotationToDistance(inputs.rotation, ElevatorConstants.drumRadius);
    inputs.velocity =
        drumVelocityToLinearVelocity(inputs.angularVelocity, ElevatorConstants.drumRadius);
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.current = currentAmps.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setPosition(Angle position) {
    motor.setControl(positionRequest.withPosition(position));
  }
}
