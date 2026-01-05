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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX primaryMotor;
  private final TalonFX secondaryMotor;

  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final VoltageOut voltageRequest = new VoltageOut(Volts.of(0));
  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(Rotations.of(0));

  public ElevatorIOTalonFX() {
    primaryMotor = ElevatorConstants.primaryMotorCanId.getTalon();
    secondaryMotor = ElevatorConstants.secondaryMotorCanId.getTalon();

    positionRot = primaryMotor.getPosition();
    velocityRotPerSec = primaryMotor.getVelocity();
    appliedVolts = primaryMotor.getMotorVoltage();
    currentAmps = primaryMotor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.CurrentLimits.withSupplyCurrentLimit(ElevatorConstants.currentLimit);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.withStatorCurrentLimit(ElevatorConstants.statorLimit);
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.motorReduction;
    config.MotorOutput.Inverted = ElevatorConstants.inverted;

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

    tryUntilOk(5, () -> primaryMotor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> secondaryMotor.getConfigurator().apply(config, 0.25));

    secondaryMotor.setControl(new Follower(primaryMotor.getDeviceID(), true));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    // TODO: fix when ctre does primaryMotor.optimizeBusUtilization();
    // TODO: fix when ctre does secondaryMotor.optimizeBusUtilization();
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
    primaryMotor.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setPosition(Angle position) {
    primaryMotor.setControl(positionRequest.withPosition(position));
  }
}
