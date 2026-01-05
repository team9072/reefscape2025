// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;

public class Robot {
  public static class Subsystems {
    // Subsystems
    public final Elevator elevator;

    Subsystems(Elevator elevator) {
      this.elevator = elevator;
    }
  }

  // Controllers
  private final CommandXboxController mainController = new CommandXboxController(0);
  // private final CommandXboxController sysIdController = new CommandXboxController(3);

  private final Subsystems s;

  public Robot() {
    final Elevator elevator;

    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        elevator = new Elevator(new ElevatorIOTalonFX());
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        elevator = new Elevator(new ElevatorIOSim());
      }

      default -> {
        // Replayed robot, disable IO implementations
        elevator = new Elevator(new ElevatorIO() {});
      }
    }

    s = new Subsystems(elevator);

    configureBindings();
  }

  private void configureBindings() {
    s.elevator.setDefaultCommand(s.elevator.moveJoystick(() -> -mainController.getLeftY() * 30));
  }

  public Command getAutonomousCommand() {
    return Commands.none();
  }
}
