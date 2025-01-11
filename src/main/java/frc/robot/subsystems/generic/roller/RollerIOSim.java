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

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
  private final DCMotorSim sim;

  private final MutVoltage appliedVoltage = Volts.mutable(0);

  public RollerIOSim(RollerConstants constants) {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.004, constants.motorReduction),
            DCMotor.getCIM(1));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    sim.setInputVoltage(appliedVoltage.in(Volts));
    sim.update(0.02);

    inputs.position = Radians.of(sim.getAngularPositionRad());
    inputs.velocity = RadiansPerSecond.of(sim.getAngularVelocityRadPerSec());
    inputs.appliedVoltage = appliedVoltage.copy();
    inputs.current = Amps.of(sim.getCurrentDrawAmps());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    appliedVoltage.mut_replace(MathUtil.clamp(voltage.in(Volts), -12.0, 12.0), Volts);
  }
}
