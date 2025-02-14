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

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim extends RollerIOTalonFX {
  private final DCMotorSim sim;
  private final TalonFXSimState simState;

  private final double motorReduction;

  public RollerIOSim(RollerConstants constants) {
    super(constants);

    motorReduction = constants.motorReduction;
    simState = roller.getSimState();

    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(1), 0.004, constants.motorReduction),
            DCMotor.getCIM(1));
  }

  @Override
  public void updateInputs(RollerIOInputs inputs) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(0.02);

    simState.setRawRotorPosition(motorReduction * sim.getAngularPositionRotations());
    simState.setRotorVelocity(
        motorReduction * Units.radiansToRotations(sim.getAngularVelocityRadPerSec()));

    super.updateInputs(inputs);
  }
}
