// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.CoralPlacer;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.autos.Autos;
import frc.robot.commands.scoring.ScoringManager;
import frc.robot.commands.scoring.ScoringMemory;
import frc.robot.commands.scoring.ScoringPosition;
import frc.robot.commands.utils.RumbleCommands;
import frc.robot.commands.utils.RumbleCommands.Rumble;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOTalonFX;
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
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.questnav.QuestNav;
import frc.robot.subsystems.questnav.QuestNavIO;
import frc.robot.subsystems.questnav.QuestNavIOReal;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants.CameraData;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class Robot {
  public static class Subsystems {
    // Subsystems
    public final Drive drive;
    public final Intake intake;
    public final Climber climber;
    public final Vision vision;
    public final QuestNav questNav;
    public final Elevator elevator;

    // State
    public final ScoringManager scoring;

    Subsystems(
        Drive drive,
        Intake intake,
        CoralPlacerPivot coralPlacerPivot,
        CoralPlacerRollers coralPlacerRollers,
        Elevator elevator,
        Climber climber,
        Vision vision,
        QuestNav questNav) {
      this.drive = drive;
      this.intake = intake;
      this.vision = vision;
      this.questNav = questNav;
      this.climber = climber;
      this.elevator = elevator;

      scoring = new ScoringManager(elevator, new CoralPlacer(coralPlacerPivot, coralPlacerRollers));
    }
  }

  // Controllers
  private final CommandXboxController mainController = new CommandXboxController(0);
  private final CommandXboxController secondaryController = new CommandXboxController(1);
  // private final CommandXboxController sysIdController = new CommandXboxController(3);

  private final Subsystems s;
  private final frc.robot.commands.autos.Autos autos;
  private final ScoringMemory scoringMemory;

  private final SendableChooser<String> modeChooser = new SendableChooser<>();
  private final Trigger driverMode = new Trigger(() -> modeChooser.getSelected().equals("driver"));
  private final Trigger demoMode = new Trigger(() -> modeChooser.getSelected().equals("demo"));

  private final Leds leds = new Leds();

  public Robot() {
    final Drive drive;
    final Intake intake;
    final CoralPlacerPivot coralPlacerPivot;
    final CoralPlacerRollers coralPlacerRollers;
    final Elevator elevator;
    final Climber climber;
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

        climber = new Climber(new ClimberIOTalonFX());

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {});

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

        climber = new Climber(new ClimberIO() {});

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

        climber = new Climber(new ClimberIO() {});

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
            drive,
            intake,
            coralPlacerPivot,
            coralPlacerRollers,
            elevator,
            climber,
            vision,
            questNav);
    autos = new Autos(s);
    scoringMemory = new ScoringMemory(ScoringPosition.branchL4);

    configureBindings();
  }

  private void configureBindings() {
    modeChooser.addOption("Advanced Driver Controls", "driver");
    modeChooser.setDefaultOption("Demo Controls", "demo");

    modeChooser.onChange(
        (_mode) -> {
          s.drive.removeDefaultCommand();
          s.intake.removeDefaultCommand();
          s.climber.removeDefaultCommand();
          s.elevator.removeDefaultCommand();

          leds.removeDefaultCommand();
          Command c = leds.getCurrentCommand();

          if (c != null) {
            c.cancel();
          }

          if (driverMode.getAsBoolean()) {
            driverBindings();
          } else {
            demoBindings();
          }
        });

    SmartDashboard.putData("Demo Mode Chooser", modeChooser);

    Command generateAutos =
        Commands.run(autos::update).ignoringDisable(true).withName("GenerateAutos");
    RobotModeTriggers.disabled().whileTrue(generateAutos);
    generateAutos.schedule();
  }

  private void driverBindings() {
    leds.setDefaultCommand(leds.driverAnimation());

    RobotModeTriggers.teleop()
        .and(driverMode)
        .and(s.intake.coralStaged)
        .debounce(0.05)
        .and(s.scoring.hasObjectAssumeTrue.negate())
        .onTrue(s.scoring.grabCoral().alongWith(scoringMemory.restoreCoralPosition()));

    s.scoring.runRollers(
        () -> scoringMemory.getMemorizedPosition().isAlgae() && driverMode.getAsBoolean());

    /** Driver Controls */
    DoubleSupplier driveXSupplier = () -> -mainController.getLeftY();
    DoubleSupplier driveYSupplier = () -> -mainController.getLeftX();
    DoubleSupplier driveOmegaSupplier = () -> -mainController.getRightX();
    double intakeDrivePercent = 0.5;
    Rumble mainControllerRumble = new Rumble(0.75);

    s.intake
        .coralStaged
        .and(driverMode)
        .debounce(0.05)
        .onTrue(RumbleCommands.rumble(mainController, mainControllerRumble).withTimeout(0.5));

    s.drive.setDefaultCommand(
        DriveCommands.joystickDrive(s.drive, driveXSupplier, driveYSupplier, driveOmegaSupplier));

    mainController
        .start()
        .and(driverMode)
        .onTrue(DriveCommands.zeroGyro(s.drive).ignoringDisable(true));

    mainController
        .leftBumper()
        .and(driverMode)
        .whileTrue(
            Commands.sequence(
                    s.scoring.clearElevator().asProxy(),
                    s.intake.intake().alongWith(s.scoring.prepForGrab().asProxy()))
                .alongWith(
                    DriveCommands.joystickDriveAtPercent(
                        s.drive,
                        intakeDrivePercent,
                        driveXSupplier,
                        driveYSupplier,
                        driveOmegaSupplier)));

    mainController.rightBumper().and(driverMode).whileTrue(s.intake.reverse());

    Trigger alignTrigger = mainController.leftTrigger().and(driverMode);
    alignTrigger.onTrue(
        s.scoring.selfCancellingScoringActionOnTrigger(
            scoringMemory::getMemorizedPosition, alignTrigger.negate()));

    BooleanSupplier algaeModifier = mainController.povDown().and(driverMode);
    Trigger scoreTrigger = mainController.rightTrigger().and(driverMode);
    scoreTrigger.onTrue(
        s.scoring.selfCancellingScoringActionOnTrigger(
            () ->
                algaeModifier.getAsBoolean()
                    ? ScoringPosition.barge
                    : scoringMemory.getMemorizedPosition(),
            scoreTrigger.negate()));

    mainController
        .a()
        .and(driverMode)
        .onTrue(
            scoringMemory
                .memorizeEither(ScoringPosition.troughL1, ScoringPosition.algaeL2, algaeModifier)
                .alongWith(s.scoring.scoringAction(ScoringPosition.algaeL2).onlyIf(algaeModifier)));
    mainController
        .x()
        .and(driverMode)
        .onTrue(
            scoringMemory
                .memorizeEither(
                    ScoringPosition.branchL2, ScoringPosition.algaeGround, algaeModifier)
                .alongWith(
                    s.scoring.scoringAction(ScoringPosition.algaeGround).onlyIf(algaeModifier)));
    mainController
        .b()
        .and(driverMode)
        .onTrue(
            scoringMemory
                .memorizeEither(ScoringPosition.branchL3, ScoringPosition.processor, algaeModifier)
                .alongWith(
                    s.scoring
                        .scoringActionOnTrigger(
                            ScoringPosition.processor, mainController.b().negate())
                        .onlyIf(algaeModifier)));
    mainController
        .y()
        .and(driverMode)
        .onTrue(
            scoringMemory
                .memorizeEither(ScoringPosition.branchL4, ScoringPosition.algaeL3, algaeModifier)
                .alongWith(s.scoring.scoringAction(ScoringPosition.algaeL3).onlyIf(algaeModifier)));

    mainController
        .povUp()
        .and(driverMode)
        .onTrue(s.scoring.grabCoral().alongWith(scoringMemory.restoreCoralPosition()));

    // Schedule a new command so the old one gets interrupted
    mainController
        .povLeft()
        .and(driverMode)
        .onTrue(Commands.defer(() -> new ScheduleCommand(s.intake.toggleDeploy()), Set.of()));

    /** Operator Controls */
    secondaryController
        .rightBumper()
        .and(driverMode)
        .whileTrue(s.intake.setPosition(PivotPosition.stow).andThen(s.intake.reverse()));
    secondaryController.leftTrigger().onTrue(s.scoring.elevatorDown());

    Trigger scoreProcessorTrigger = secondaryController.leftBumper().and(driverMode);
    scoreProcessorTrigger.onTrue(
        s.scoring.scoringActionOnTrigger(
            ScoringPosition.processor, scoreProcessorTrigger.negate()));

    // Elevator unstuck
    secondaryController
        .povDown()
        .and(driverMode)
        .onTrue(s.scoring.unstuckElevator(ElevatorPosition.reefL3Position));
    secondaryController.povUp().onTrue(s.scoring.unstuckElevator(ElevatorPosition.reefL4Position));

    // Coral Placer jog
    secondaryController.povLeft().and(driverMode).onTrue(s.scoring.jogCoralPlacerUp());

    secondaryController.povRight().and(driverMode).onTrue(s.scoring.jogCoralPlacerDown());

    secondaryController.back().and(driverMode).onTrue(s.scoring.untrustAllSensors());

    s.climber.setDefaultCommand(
        ClimbCommands.manualControl(
            s, () -> MathUtil.applyDeadband(secondaryController.getRightY(), 0.1)));

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

  private void demoBindings() {
    leds.setDefaultCommand(leds.demoAnimation());

    DoubleSupplier driveXSupplier = () -> -mainController.getLeftY();
    DoubleSupplier driveYSupplier = () -> -mainController.getLeftX();
    DoubleSupplier driveOmegaSupplier = () -> -mainController.getRightX();

    s.drive.setDefaultCommand(
        DriveCommands.joystickDriveAtPercent(
            s.drive, 0.2, driveXSupplier, driveYSupplier, driveOmegaSupplier));

    s.intake.setDefaultCommand(s.intake.setPosition(PivotPosition.deploy));

    RobotModeTriggers.teleop()
        .and(demoMode)
        .and(s.intake.coralStaged)
        .debounce(0.05)
        .and(s.scoring.hasObjectAssumeFalse.negate())
        .onTrue(
            Commands.sequence(
                s.scoring.grabCoral(),
                s.scoring.scoringAction(ScoringPosition.troughL1),
                s.scoring.elevatorDown()));

    mainController.b().and(demoMode).whileTrue(s.intake.intake());
    mainController.a().and(demoMode).whileTrue(s.intake.reverse());

    s.elevator.setDefaultCommand(s.scoring.elevatorDown());
  }

  public Command getAutonomousCommand() {
    return Commands.none(); // autos.selectedAuto();
  }
}
