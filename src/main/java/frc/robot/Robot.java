// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Autos;
import frc.robot.commands.CoralFlow;
import frc.robot.commands.CoralFlow.ReefBranch;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignment;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacer;
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
import frc.robot.subsystems.generic.beambreak.BeamBreakIO;
import frc.robot.subsystems.generic.beambreak.BeamBreakIODio;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOSim;
import frc.robot.subsystems.generic.roller.RollerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.PivotIO;
import frc.robot.subsystems.intake.PivotIOSim;
import frc.robot.subsystems.intake.PivotIOTalonFX;
import frc.robot.subsystems.questnav.QuestNav;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraData;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;

public class Robot {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final CoralPlacer coralPlacer;
  private final Elevator elevator;
  private final Vision vision;
  private final QuestNav questNav;

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  // private final CommandXboxController sysIdController = new CommandXboxController(3);

  // Autonomous
  private final Autos autos;
  private final CoralFlow coralFlow;

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
                new RollerIOTalonFX(IntakeConstants.stagingRoller),
                new BeamBreakIODio(IntakeConstants.beamBreakDioId));

        coralPlacer = new CoralPlacer(new CoralPlacerIOTalonFX());

        elevator = new Elevator(new ElevatorIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(CameraData.LeftCamera),
                new VisionIOPhotonVision(CameraData.RightCamera));

        questNav = new QuestNav(new QuestNavIOReal(), drive::addVisionMeasurement);
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
                new RollerIOSim(IntakeConstants.stagingRoller),
                new BeamBreakIO() {});

        coralPlacer = new CoralPlacer(new CoralPlacerIOSim());

        elevator = new Elevator(new ElevatorIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(CameraData.LeftCamera, drive::getPose),
                new VisionIOPhotonVisionSim(CameraData.RightCamera, drive::getPose));

        questNav = new QuestNav(new QuestNavIO() {}, drive::addVisionMeasurement);
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

        coralPlacer = new CoralPlacer(new CoralPlacerIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake =
            new Intake(
                new PivotIO() {},
                new RollerIO() {},
                new RollerIO() {},
                new RollerIO() {},
                new BeamBreakIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        questNav = new QuestNav(new QuestNavIO() {}, drive::addVisionMeasurement);
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

    controller.leftBumper().whileTrue(elevator.clearCoral().andThen(intake.intake()));

    // Schedule a new command so the old one gets interrupted
    controller
        .rightBumper()
        .onTrue(
            Commands.defer(
                () ->
                    new ScheduleCommand(
                        intake.toggleDeploy(elevator.setPosition(ElevatorPosition.readyPosition))),
                Set.of()));

    controller
        .leftTrigger()
        .whileTrue(
            ReefAlignment.driveReefAligned(
                drive, () -> -controller.getLeftY(), () -> -controller.getLeftX()));

    controller.a().whileTrue(intake.reverse());

    controller.povDown().onTrue(coralFlow.grabCoral());
    controller.x().onTrue(coralFlow.memorizeBranch(ReefBranch.branchL2));
    controller.b().onTrue(coralFlow.memorizeBranch(ReefBranch.branchL3));
    controller.y().onTrue(coralFlow.memorizeBranch(ReefBranch.branchL4));

    Trigger scoreTrigger = controller.rightTrigger();
    scoreTrigger.onTrue(
        coralFlow.scoreCoralOnTrigger(coralFlow::getMemorizedBranch, scoreTrigger.negate()));

    // Reset gyro to 0° when start button is pressed
    controller.start().onTrue(DriveCommands.zeroGyro(drive).ignoringDisable(true));

    // SysId Controls
    /*sysIdController.leftBumper().onTrue(Commands.runOnce(() -> SignalLogger.start()));

    sysIdController.rightBumper().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

    sysIdController.povUp().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysIdController.povDown().whileTrue(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    sysIdController.povLeft().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIdController.povRight().whileTrue(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    sysIdController.a().whileTrue(DriveCommands.feedforwardCharacterization(drive));
    sysIdController
        .b()
        .whileTrue(
            Commands.run(
                () -> drive.runVelocity(new ChassisSpeeds(-sysIdController.getLeftY(), 0, 0))));*/
  }

  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }
}
