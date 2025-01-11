package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static frc.robot.util.UnitCalculations.drumRotationToDistance;
import static frc.robot.util.UnitCalculations.drumVelocityToLinearVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOTalonFX implements ElevatorIO {
  private TalonFX motor;
  private final StatusSignal<Angle> positionRot = motor.getPosition();
  private final StatusSignal<AngularVelocity> velocityRotPerSec = motor.getVelocity();
  private final StatusSignal<Voltage> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<Current> currentAmps = motor.getSupplyCurrent();

  public ElevatorIOTalonFX() {
    motor = ElevatorConstants.motorCanId.getTalon();

    var config = new TalonFXConfiguration();

    config.CurrentLimits.withSupplyCurrentLimit(ElevatorConstants.currentLimit);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Feedback.SensorToMechanismRatio = ElevatorConstants.motorReduction;

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.position = drumRotationToDistance(positionRot.getValue(), ElevatorConstants.drumRadius);
    inputs.velocity =
        drumVelocityToLinearVelocity(velocityRotPerSec.getValue(), ElevatorConstants.drumRadius);
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.current = currentAmps.getValue();
  }
}
