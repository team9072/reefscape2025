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
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
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
import java.util.OptionalDouble;
import java.util.Set;

public class Robot {
  public static class Subsystems {
    // Subsystems
    public final Drive drive;
    public final Intake intake;
    public final Vision vision;
    public final QuestNav questNav;

    // State
    public final CoralFlow coralFlow;

    Subsystems(
        Drive drive,
        Intake intake,
        CoralPlacer coralPlacer,
        Elevator elevator,
        Vision vision,
        QuestNav questNav) {
      this.drive = drive;
      this.intake = intake;
      this.vision = vision;
      this.questNav = questNav;

      coralFlow = new CoralFlow(intake, elevator, coralPlacer);
    }
  }

  // Controllers
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  // private final CommandXboxController sysIdController = new CommandXboxController(3);

  private final Subsystems s;
  private final Autos autos;

  public Robot() {
    final Drive drive;
    final Intake intake;
    final CoralPlacer coralPlacer;
    final Elevator elevator;
    final Vision vision;
    final QuestNav questNav;

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

    s = new Subsystems(drive, intake, coralPlacer, elevator, vision, questNav);
    autos = new Autos(s);

    configureBindings();
  }

  private void configureBindings() {
    /** Driver Controls */
    s.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            s.drive,
            () -> -mainController.getLeftY(),
            () -> -mainController.getLeftX(),
            () -> -mainController.getRightX()));

    mainController
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDrive(
                s.drive,
                () -> -mainController.getLeftY(),
                () -> -mainController.getLeftX(),
                () -> -mainController.getRightX(),
                () -> OptionalDouble.of(0.15)));

    mainController.start().onTrue(DriveCommands.zeroGyro(s.drive).ignoringDisable(true));

    s.intake.setDefaultCommand(s.intake.idleStagingRoller());

    mainController.leftBumper().whileTrue(s.coralFlow.clearElevator().andThen(s.intake.intake()));

    // Schedule a new command so the old one gets interrupted
    mainController
        .rightBumper()
        .onTrue(
            Commands.defer(
                () -> new ScheduleCommand(s.intake.toggleDeploy(s.coralFlow.elevatorDown())),
                Set.of()));

    mainController.a().whileTrue(s.intake.reverse());

    Trigger scoreTrigger = mainController.rightTrigger();
    scoreTrigger.onTrue(
        s.coralFlow.scoreCoralOnTrigger(s.coralFlow::getMemorizedBranch, scoreTrigger.negate()));

    mainController.x().onTrue(s.coralFlow.memorizeBranch(ReefBranch.branchL2));
    mainController.b().onTrue(s.coralFlow.memorizeBranch(ReefBranch.branchL3));
    mainController.y().onTrue(s.coralFlow.memorizeBranch(ReefBranch.branchL4));

    // Mapped to back buttons
    mainController.povDown().onTrue(s.coralFlow.grabCoral());

    /** Operator Controls */
    secondaryController
        .rightBumper()
        .whileTrue(s.intake.setPosition(PivotPosition.stow).andThen(s.intake.reverse()));
    secondaryController
        .leftBumper()
        .whileTrue(
            s.intake.setPosition(PivotPosition.unjam).withTimeout(0.4).andThen(s.intake.reverse()));

    secondaryController.rightTrigger().onTrue(s.coralFlow.prepareElevator(ReefBranch.branchL2));

    secondaryController
        .leftTrigger()
        .whileTrue(
            s.intake
                .setPosition(PivotPosition.algae)
                .withTimeout(0)
                .andThen(s.intake.intakeAlgae()));
    secondaryController.leftTrigger().onFalse(s.intake.reverseAlgae().withTimeout(1));

    secondaryController
        .a()
        .onTrue(
            s.coralFlow.prepareElevator(ReefBranch.branchL2).andThen(s.coralFlow.removeAlgae()));
    secondaryController
        .y()
        .onTrue(
            s.coralFlow.prepareElevator(ReefBranch.branchL3).andThen(s.coralFlow.removeAlgae()));

    secondaryController.x().onTrue(s.coralFlow.knockCoralOff());

    /** SysId Controls */
    /*sysIdController.leftBumper().onTrue(Commands.runOnce(() -> SignalLogger.start()));

    sysIdController.rightBumper().onTrue(Commands.runOnce(() -> SignalLogger.stop()));

    sysIdController.povUp().whileTrue(s.drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    sysIdController.povDown().whileTrue(s.drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    sysIdController.povLeft().whileTrue(s.drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    sysIdController.povRight().whileTrue(s.drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    sysIdController.a().whileTrue(DriveCommands.feedforwardCharacterization(s.drive));
    sysIdController
        .b()
        .whileTrue(
            Commands.run(
                () -> s.drive.runVelocity(new ChassisSpeeds(-sysIdController.getLeftY(), 0, 0))));*/
  }

  public Command getAutonomousCommand() {
    return autos.getSelectedAuto();
  }
}
