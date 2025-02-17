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

package frc.robot.subsystems.coralplacer;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class CoralPlacerIOTalonFX implements CoralPlacerIO {
  protected final TalonFX motor;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final PositionVoltage floatRequest = new PositionVoltage(0).withSlot(0);
  private final MotionMagicVoltage deployRequest = new MotionMagicVoltage(0).withSlot(1);

  public CoralPlacerIOTalonFX() {
    motor = CoralPlacerConstants.motorCanId.getTalon();
    positionRot = motor.getPosition();
    velocityRotPerSec = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    currentAmps = motor.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.CurrentLimits.withSupplyCurrentLimit(CoralPlacerConstants.currentLimit);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Feedback.SensorToMechanismRatio = CoralPlacerConstants.motorReduction;

    // Floating
    config.withSlot0(
        new Slot0Configs()
            .withKG(CoralPlacerConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine));

    config.withSlot1(
        new Slot1Configs()
            .withKP(CoralPlacerConstants.kP)
            .withKD(CoralPlacerConstants.kD)
            .withKV(CoralPlacerConstants.kV)
            .withKG(CoralPlacerConstants.kG)
            .withGravityType(GravityTypeValue.Arm_Cosine));

    config.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(CoralPlacerConstants.rampVelocity)
            .withMotionMagicAcceleration(CoralPlacerConstants.rampAcceleration));

    tryUntilOk(5, () -> motor.getConfigurator().apply(config, 0.25));
    tryUntilOk(5, () -> motor.setPosition(PivotPosition.stow.angle, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(CoralPlacerInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.current = currentAmps.getValue();
  }

  @Override
  public void setPosition(Angle positon) {
    floatRequest.withPosition(positon);
    motor.setControl(deployRequest.withPosition(positon));
  }
}
