// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.climber.ClimberConstants.ClimberPosition;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class ClimberIOTalonFX implements ClimberIO {
  protected final TalonFX motor;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
  private final VoltageOut manualControlRequest = new VoltageOut(0);

  public ClimberIOTalonFX() {
    motor = ClimberConstants.canId.getTalon();
    positionRot = motor.getPosition();
    velocityRotPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.MotorOutput.NeutralMode = ClimberConstants.neutralMode;
    config.MotorOutput.Inverted = ClimberConstants.invertedValue;

    config.Feedback.SensorToMechanismRatio = ClimberConstants.motorReduction;

    config.withSlot0(
        new Slot0Configs()
            .withKP(ClimberConstants.kP)
            .withKD(ClimberConstants.kD)
            .withKV(ClimberConstants.kV)
            .withKG(ClimberConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine));

    config.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(ClimberConstants.rampVelocity)
            .withMotionMagicAcceleration(ClimberConstants.rampAcceleration));

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(ClimberPosition.stowPosition.angle, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    // TODO: fix when ctre does motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ClimberInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.current = currentAmps.getValue();
  }

  @Override
  public void setPosition(Angle positon) {
    motor.setControl(positionRequest.withPosition(positon));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setControl(manualControlRequest.withOutput(voltage));
  }
}
