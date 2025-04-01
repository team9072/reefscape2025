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
import frc.robot.commands.CoralPlacer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.scoring.ScoringManager;
import frc.robot.commands.scoring.ScoringMemory;
import frc.robot.commands.scoring.ScoringPosition;
import frc.robot.commands.utils.RumbleCommands;
import frc.robot.commands.utils.RumbleCommands.Rumble;
import frc.robot.commands.utils.TapHold;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacerIO;
import frc.robot.subsystems.coralplacer.CoralPlacerIOSim;
import frc.robot.subsystems.coralplacer.CoralPlacerIOTalonFX;
import frc.robot.subsystems.coralplacer.CoralPlacerPivot;
import frc.robot.subsystems.coralplacer.CoralPlacerRollers;
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
import java.util.Set;
import java.util.function.DoubleSupplier;

public class Robot {
  public static class Subsystems {
    // Subsystems
    public final Drive drive;
    public final Intake intake;
    public final Vision vision;
    public final QuestNav questNav;

    // State
    public final ScoringManager scoring;

    Subsystems(
        Drive drive,
        Intake intake,
        CoralPlacerPivot coralPlacerPivot,
        CoralPlacerRollers coralPlacerRollers,
        Elevator elevator,
        Vision vision,
        QuestNav questNav) {
      this.drive = drive;
      this.intake = intake;
      this.vision = vision;
      this.questNav = questNav;

      scoring = new ScoringManager(elevator, new CoralPlacer(coralPlacerPivot, coralPlacerRollers));
    }
  }

  // Controllers
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  // private final CommandXboxController sysIdController = new CommandXboxController(3);

  private final Subsystems s;
  private final Autos autos;
  private final ScoringMemory scoringMemory;

  public Robot() {
    final Drive drive;
    final Intake intake;
    final CoralPlacerPivot coralPlacerPivot;
    final CoralPlacerRollers coralPlacerRollers;
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
                new BeamBreakIODio(IntakeConstants.passthroughBeamBreakDioId),
                new BeamBreakIODio(IntakeConstants.stagingBeamBreakDioId));

        coralPlacerPivot =
            new CoralPlacerPivot(
                new CoralPlacerIOTalonFX(), new RollerIOTalonFX(CoralPlacerConstants.roller));

        coralPlacerRollers =
            new CoralPlacerRollers(
                new RollerIOTalonFX(CoralPlacerConstants.roller),
                new BeamBreakIODio(CoralPlacerConstants.beamBreakDioId));

        elevator = new Elevator(new ElevatorIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(CameraData.RightCamera),
                new VisionIOPhotonVision(CameraData.LeftCamera));

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
                new BeamBreakIO() {},
                new BeamBreakIO() {});

        coralPlacerPivot =
            new CoralPlacerPivot(
                new CoralPlacerIOSim(), new RollerIOSim(CoralPlacerConstants.roller));

        coralPlacerRollers =
            new CoralPlacerRollers(
                new RollerIOSim(CoralPlacerConstants.roller), new BeamBreakIO() {});

        elevator = new Elevator(new ElevatorIOSim());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(CameraData.RightCamera, drive::getPose),
                new VisionIOPhotonVisionSim(CameraData.LeftCamera, drive::getPose));

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

        coralPlacerPivot = new CoralPlacerPivot(new CoralPlacerIO() {}, new RollerIO() {});

        coralPlacerRollers = new CoralPlacerRollers(new RollerIO() {}, new BeamBreakIO() {});

        elevator = new Elevator(new ElevatorIO() {});

        intake =
            new Intake(
                new PivotIO() {},
                new RollerIO() {},
                new RollerIO() {},
                new BeamBreakIO() {},
                new BeamBreakIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        questNav = new QuestNav(new QuestNavIO() {}, drive::addVisionMeasurement);
      }
    }

    s =
        new Subsystems(
            drive, intake, coralPlacerPivot, coralPlacerRollers, elevator, vision, questNav);
    autos = new Autos(s);
    scoringMemory = new ScoringMemory(ScoringPosition.branchL4);

    configureBindings();
  }

  private void configureBindings() {
    /** Driver Controls */
    DoubleSupplier driveXSupplier = () -> -mainController.getLeftY();
    DoubleSupplier driveYSupplier = () -> -mainController.getLeftX();
    DoubleSupplier driveOmegaSupplier = () -> -mainController.getRightX();
    double intakeDrivePercent = 0.45;
    Rumble mainControllerRumble = new Rumble(0.75);

    s.intake
        .coralStaged
        .debounce(0.05)
        .onTrue(RumbleCommands.rumble(mainController, mainControllerRumble).withTimeout(0.5));

    s.drive.setDefaultCommand(
        DriveCommands.joystickDrive(s.drive, driveXSupplier, driveYSupplier, driveOmegaSupplier));

    mainController.start().onTrue(DriveCommands.zeroGyro(s.drive).ignoringDisable(true));

    mainController
        .leftBumper()
        .whileTrue(
            Commands.sequence(
                    s.scoring.clearElevator().asProxy(),
                    s.intake.intake().alongWith(s.scoring.elevatorDown().asProxy()))
                .alongWith(
                    DriveCommands.joystickDriveAtPercent(
                        s.drive,
                        intakeDrivePercent,
                        driveXSupplier,
                        driveYSupplier,
                        driveOmegaSupplier)));

    mainController.rightBumper().whileTrue(s.intake.reverse());

    Trigger scoreTrigger = mainController.rightTrigger();
    scoreTrigger.onTrue(
        s.scoring.scoringActionOnTrigger(
            scoringMemory::getMemorizedPosition, scoreTrigger.negate()));

    Trigger alignTrigger = mainController.leftTrigger();
    alignTrigger.onTrue(
        s.scoring.scoringActionOnTrigger(
            scoringMemory::getMemorizedPosition, alignTrigger.negate()));
    alignTrigger.whileTrue(
        ReefAlignment.driveReefAligned(
            s.drive, driveXSupplier, driveYSupplier, scoringMemory::getMemorizedPosition));

    mainController.a().onTrue(scoringMemory.memorizePosition(ScoringPosition.troughL1));
    mainController.x().onTrue(scoringMemory.memorizePosition(ScoringPosition.branchL2));
    mainController.b().onTrue(scoringMemory.memorizePosition(ScoringPosition.branchL3));
    mainController.y().onTrue(scoringMemory.memorizePosition(ScoringPosition.branchL4));

    // D-Pad is mapped to back buttons
    mainController
        .povDown()
        .onTrue(s.scoring.grabCoral().alongWith(scoringMemory.restoreCoralPosition()));

    // Schedule a new command so the old one gets interrupted
    mainController
        .povLeft()
        .onTrue(Commands.defer(() -> new ScheduleCommand(s.intake.toggleDeploy()), Set.of()));

    /** Operator Controls */
    secondaryController
        .rightBumper()
        .whileTrue(s.intake.setPosition(PivotPosition.stow).andThen(s.intake.reverse()));
    secondaryController.leftTrigger().whileTrue(s.scoring.elevatorDown());

    secondaryController
        .a()
        .onTrue(s.scoring.prepareElevator(ScoringPosition.algaeL2))
        .onFalse(s.scoring.scoringAction(ScoringPosition.algaeL2));
    secondaryController
        .y()
        .onTrue(s.scoring.prepareElevator(ScoringPosition.algaeL2))
        .onFalse(s.scoring.scoringAction(ScoringPosition.algaeL3));

    secondaryController
        .b()
        .onTrue(
            s.scoring
                .prepareElevator(ScoringPosition.algaeGround)
                .alongWith(
                    DriveCommands.joystickDriveAtPercent(
                        s.drive,
                        intakeDrivePercent,
                        driveXSupplier,
                        driveYSupplier,
                        driveOmegaSupplier)))
        .onFalse(s.scoring.scoringAction(ScoringPosition.algaeGround));

    secondaryController.x().onTrue(s.scoring.unstuckCoralPlacer());

    TapHold algaeControl = new TapHold(secondaryController.rightTrigger(), 0.5);
    algaeControl.tap.onTrue(s.scoring.stowHold());

    Trigger scoreBargeTrigger = algaeControl.hold;
    scoreBargeTrigger.onTrue(
        s.scoring.scoringActionOnTrigger(ScoringPosition.barge, scoreBargeTrigger.negate()));

    Trigger scoreProcessorTrigger = secondaryController.leftBumper();
    scoreProcessorTrigger.onTrue(
        s.scoring.scoringActionOnTrigger(
            ScoringPosition.processor, scoreProcessorTrigger.negate()));

    // Elevator unstuck
    secondaryController
        .povDown()
        .onTrue(s.scoring.unstuckElevator(ElevatorPosition.reefL3Position));
    secondaryController.povUp().onTrue(s.scoring.unstuckElevator(ElevatorPosition.reefL4Position));

    // Coral Placer jog
    secondaryController.povLeft().onTrue(s.scoring.jogCoralPlacerUp());

    secondaryController.povRight().onTrue(s.scoring.jogCoralPlacerDown());

    secondaryController.back().onTrue(s.scoring.untrustAllSensors());

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
