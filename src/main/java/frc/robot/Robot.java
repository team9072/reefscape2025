// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralFlow;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacer;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.coralplacer.CoralPlacerIO;
import frc.robot.subsystems.coralplacer.CoralPlacerIOSim;
import frc.robot.subsystems.coralplacer.CoralPlacerIOTalonFX;
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
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOSim;
import frc.robot.subsystems.generic.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOTalonFX;

public class Robot {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  /*private final Vision vision;*/
  private final CoralPlacer coralPlacer;
  private final Elevator elevator;

  private final CoralFlow coralFlow;

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

        intake =
            new Intake(
                new PivotIOTalonFX(IntakeConstants.pivot),
                new RollerIOTalonFX(IntakeConstants.roller),
                new RollerIOTalonFX(IntakeConstants.pasthrough),
                new RollerIOTalonFX(IntakeConstants.stagingRoller));

        /*vision =
        new Vision(
            drive::addVisionMeasurement, new VisionIOPhotonVision(CameraData.LeftCamera));*/

        coralPlacer = new CoralPlacer(new CoralPlacerIOTalonFX());

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

        intake =
            new Intake(
                new PivotIOSim(IntakeConstants.pivot),
                new RollerIOSim(IntakeConstants.roller),
                new RollerIOSim(IntakeConstants.pasthrough),
                new RollerIOSim(IntakeConstants.stagingRoller));

        /*vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(CameraData.LeftCamera, drive::getPose));*/

        coralPlacer = new CoralPlacer(new CoralPlacerIOSim());

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

        intake =
            new Intake(new PivotIO() {}, new RollerIO() {}, new RollerIO() {}, new RollerIO() {});

        /*vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});*/

        coralPlacer = new CoralPlacer(new CoralPlacerIO() {});

        elevator = new Elevator(new ElevatorIO() {});
      }
    }

    autos = new Autos(drive);
    coralFlow = new CoralFlow(intake, elevator, coralPlacer);

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

    intake.setDefaultCommand(intake.idleStagingRoller());

    controller.a().whileTrue(elevator.clearCoral().andThen(intake.intake()));
    controller.y().whileTrue(elevator.clearCoral().andThen(intake.reverse()));
    controller.x().whileTrue(intake.intakeAlgae());

    controller
        .rightBumper()
        .onTrue(
            intake.toggleDeploy(
                elevator.setPosition(ElevatorPosition.readyPosition).withTimeout(0)));

    /*controller
        .leftTrigger()
        .whileTrue(
            ReefAlignment.driveReefAligned(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.a().whileTrue(coralPlacer.extend());*/

    controller.povDown().onTrue(coralFlow.grabCoral());
    controller.povLeft().onTrue(elevator.setPosition(ElevatorPosition.reefL2Position));
    controller.povRight().onTrue(elevator.setPosition(ElevatorPosition.reefL3Position));
    controller.povUp().onTrue(elevator.setPosition(ElevatorPosition.reefL4Position));

    Trigger scoreTrigger = controller.rightTrigger().debounce(0.2);
    /*scoreTrigger.whileTrue(
    coralPlacer
        .setPosition(CoralPlacerPosition.readyPosition)
        .andThen(
            Commands.waitUntil(scoreTrigger.negate()),
            coralPlacer.setPosition(CoralPlacerPosition.scorePosition)));*/
    scoreTrigger.onTrue(coralPlacer.setPosition(CoralPlacerPosition.scorePosition));

    // Reset gyro to 0° when start button is pressed
    controller.start().onTrue(DriveCommands.zeroGyro(drive).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }
}
