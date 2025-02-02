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

package frc.robot.subsystems.generic.roller;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/**
 * This roller implementation is for a Talon FX driving a motor like the Falon 500 or Kraken X60.
 */
public class RollerIOTalonFX implements RollerIO {
  protected final TalonFX roller;
  private final StatusSignal<Angle> positionRot;
  private final StatusSignal<AngularVelocity> velocityRotPerSec;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> currentAmps;

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final TorqueCurrentFOC torqueRequest = new TorqueCurrentFOC(0.0);

  public RollerIOTalonFX(RollerConstants constants) {
    roller = constants.canId.getTalon();
    positionRot = roller.getPosition();
    velocityRotPerSec = roller.getVelocity();
    appliedVolts = roller.getMotorVoltage();
    currentAmps = roller.getSupplyCurrent();

    var config = new TalonFXConfiguration();

    config.CurrentLimits.withSupplyCurrentLowerLimit(constants.baseCurrentLimit);
    config.CurrentLimits.withSupplyCurrentLimit(constants.spikeCurrentLimit);
    config.CurrentLimits.withSupplyCurrentLowerTime(constants.spikeTime);
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.CurrentLimits.withStatorCurrentLimit(constants.statorCurrentLimit);
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = constants.neutralMode;

    tryUntilOk(5, () -> roller.getConfigurator().apply(config, 0.25));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, positionRot, velocityRotPerSec, appliedVolts, currentAmps);
    roller.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    BaseStatusSignal.refreshAll(positionRot, velocityRotPerSec, appliedVolts, currentAmps);

    inputs.position = positionRot.getValue();
    inputs.velocity = velocityRotPerSec.getValue();
    inputs.appliedVoltage = appliedVolts.getValue();
    inputs.current = currentAmps.getValue();
  }

  @Override
  public void setVoltage(Voltage voltage) {
    roller.setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setTorque(Current current, double maxDutyCycle) {
    roller.setControl(torqueRequest.withOutput(current).withMaxAbsDutyCycle(maxDutyCycle));
  }
}
