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

package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.UnitCalculations.drumRotationToDistance;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
  private final ElevatorSim elevatorSim;

  private final MutVoltage appliedVoltage = Volts.mutable(0);

  public ElevatorIOSim() {
    DCMotor motor = DCMotor.getKrakenX60(1);

    Distance minHeight =
        drumRotationToDistance(ElevatorConstants.minDistance, ElevatorConstants.drumRadius);
    Distance maxHeight =
        drumRotationToDistance(ElevatorConstants.maxDistance, ElevatorConstants.drumRadius);

    elevatorSim =
        new ElevatorSim(
            LinearSystemId.createElevatorSystem(
                motor,
                ElevatorConstants.elevatorMass.in(Kilograms),
                ElevatorConstants.drumRadius.in(Meters),
                ElevatorConstants.motorReduction),
            motor,
            minHeight.in(Meters),
            maxHeight.in(Meters),
            false,
            minHeight.in(Meters),
            null);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    elevatorSim.setInputVoltage(appliedVoltage.in(Volts));
    elevatorSim.update(0.02);

    inputs.position = Meters.of(elevatorSim.getPositionMeters());
    inputs.velocity = MetersPerSecond.of(elevatorSim.getVelocityMetersPerSecond());
    inputs.appliedVoltage = appliedVoltage.copy();
    inputs.current = Amps.of(elevatorSim.getCurrentDrawAmps());
  }
}
