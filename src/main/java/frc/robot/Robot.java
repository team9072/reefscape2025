// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacer;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.generic.beambreak.BeamBreakIO;
import frc.robot.subsystems.generic.beambreak.BeamBreakIODio;
import frc.robot.subsystems.generic.beambreak.BeamBreakIONull;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOSim;
import frc.robot.subsystems.generic.roller.RollerIOTalonFX;

public class Robot {
  // Subsystems
  private final Drive drive;
  private final CoralPlacer coralPlacer;
  private final Elevator elevator;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Autonomous
  private final Autos autos;

  public Robot() {
    switch (Constants.currentMode) {
      case REAL -> {
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        coralPlacer =
            new CoralPlacer(
                new RollerIOTalonFX(CoralPlacerConstants.leftRoller),
                new RollerIOTalonFX(CoralPlacerConstants.rightRoller),
                new BeamBreakIODio(CoralPlacerConstants.beamBreakDioId));

        elevator = new Elevator(new ElevatorIOTalonFX());
      }

      case SIM -> {
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        coralPlacer =
            new CoralPlacer(
                new RollerIOSim(CoralPlacerConstants.leftRoller),
                new RollerIOSim(CoralPlacerConstants.rightRoller),
                new BeamBreakIONull());

        elevator = new Elevator(new ElevatorIOSim());
      }

      default -> {
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        coralPlacer = new CoralPlacer(new RollerIO() {}, new RollerIO() {}, new BeamBreakIO() {});

        elevator = new Elevator(new ElevatorIO() {});
      }
    }

    autos = new Autos(drive);
    configureBindings();
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.b().whileTrue(coralPlacer.intake());
    controller.a().whileTrue(coralPlacer.extend());

    controller.povDown().onTrue(elevator.setPosition(ElevatorPosition.reefL2Position));
    controller.povUp().onTrue(elevator.setPosition(ElevatorPosition.reefL3Position));

    // Reset gyro to 0° when start button is pressed
    controller.start().onTrue(DriveCommands.zeroGyro(drive).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }
}
