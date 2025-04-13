package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralPlacer;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ScoringManager {
  private final Elevator elevator;
  private final CoralPlacer coralPlacer;

  private boolean trustSensors = true;
  public final Trigger hasObjectAssumeFalse;
  public final Trigger hasObjectAssumeTrue;
  public final Trigger coralPlacerHasObject;

  public ScoringManager(Elevator elevator, CoralPlacer coralPlacer) {
    this.elevator = elevator;
    this.coralPlacer = coralPlacer;

    coralPlacerHasObject = coralPlacer.hasObject;
    hasObjectAssumeFalse = coralPlacer.hasObject.and(() -> trustSensors);
    hasObjectAssumeTrue = coralPlacer.hasObject.or(() -> !trustSensors);
  }

  public void runRollers(BooleanSupplier doAlgaeSpeed) {
    coralPlacer.rollers().setDefaultCommand(coralPlacer.holdObject(doAlgaeSpeed));
  }

  public Command untrustAllSensors() {
    return Commands.runOnce(() -> trustSensors = false);
  }

  /** Returns the elevator to the ready position, and prepares the coral placer for a grab */
  public Command elevatorDown() {
    return Commands.sequence(
        clearElevatorIfNeeded(ElevatorPosition.grabPosition, CoralPlacerPosition.grabPosition),
        Commands.parallel(
            elevator.setPosition(ElevatorPosition.readyPosition),
            Commands.sequence(
                coralPlacer
                    .setPosition(CoralPlacerPosition.holdPosition)
                    .repeatedly()
                    .onlyIf(coralPlacer.pastScorePosition.negate())
                    .until(() -> elevator.atPosition(ElevatorPosition.readyPosition)),
                coralPlacer.setPosition(CoralPlacerPosition.grabPosition))));
  }

  /**
   * Returns the elevator to the ready position. The command only runs until the elevator is clear
   * of the staging zone
   */
  public Command clearElevator() {
    return Commands.sequence(coralPlacer.clearTop(), elevator.clearCoral());
  }

  private Command clearElevatorIfNeeded(
      ElevatorPosition elevatorPosition, CoralPlacerPosition coralPlacerPosition) {
    return Commands.sequence(
        coralPlacer
            .clearTop()
            .unless(
                () ->
                    elevatorPosition == ElevatorPosition.scoreBargePosition
                        && elevator.atPosition(ElevatorPosition.scoreBargePosition)),
        elevator.clearCoral().onlyIf(() -> coralPlacer.wouldCrossCupHitZone(coralPlacerPosition)));
  }

  private Command prepareElevator(ScoringPosition position, boolean isAuto) {
    final Command command;
    final CoralPlacerPosition coralPlacerGoal;

    if (position.isObjectIntake()) {
      // Prepare to intake algae
      coralPlacerGoal =
          position == ScoringPosition.coralGround
              ? CoralPlacerPosition.grabGroundCoralPosition
              : position == ScoringPosition.algaeGround
                  ? CoralPlacerPosition.grabGroundAlgaePosition
                  : CoralPlacerPosition.grabReefAlgaePosition;

      command =
          Commands.parallel(
                  coralPlacer.setPosition(coralPlacerGoal),
                  elevator.setPosition(position.elevatorPosition))
              .beforeStarting(
                  coralPlacer
                      .setPosition(coralPlacerGoal)
                      .onlyIf(() -> position == ScoringPosition.algaeGround));
    } else if (position.isAlgaeScore()) {
      // Prepare to score algae
      final boolean isBarge = position == ScoringPosition.barge;
      coralPlacerGoal =
          isBarge
              ? CoralPlacerPosition.scoreBargePosition
              : CoralPlacerPosition.scoreProcessorPosition;

      CoralPlacerPosition coralPlacerHold =
          isBarge
              ? CoralPlacerPosition.algaeStowPosition
              : CoralPlacerPosition.scoreProcessorPosition;

      command =
          Commands.sequence(
              coralPlacer
                  .setPositionAlgae(coralPlacerHold)
                  .unless(() -> isBarge && elevator.atPosition(position.elevatorPosition))
                  .until(() -> !isBarge && !coralPlacer.wouldCrossCupHitZone(coralPlacerHold)),
              elevator.setPosition(position.elevatorPosition),
              coralPlacer.setPositionAlgae(coralPlacerGoal));
    } else {
      // Score coral
      final boolean isL4 = position == ScoringPosition.branchL4;
      final boolean isL1 = position == ScoringPosition.troughL1;

      coralPlacerGoal =
          isL1
              ? CoralPlacerPosition.scoreTroughPosition
              : isAuto
                  ? CoralPlacerPosition.holdPosition
                  : isL4
                      ? CoralPlacerPosition.preScoreHoldPositionL4
                      : CoralPlacerPosition.preScoreHoldPosition;

      command =
          Commands.sequence(
              coralPlacer
                  .setPosition(CoralPlacerPosition.holdPosition)
                  .unless(() -> elevator.atPosition(position.elevatorPosition)),
              elevator.setPosition(position.elevatorPosition),
              coralPlacer.setPosition(coralPlacerGoal));
    }

    return Commands.sequence(
            clearElevatorIfNeeded(position.elevatorPosition, coralPlacerGoal), command)
        .deadlineFor(coralPlacer.holdAlgae().onlyIf(position::isAlgae))
        .unless(
            () ->
                elevator.atPosition(position.elevatorPosition)
                    && coralPlacer.atPosition(coralPlacerGoal));
  }

  public Command prepareElevator(ScoringPosition position) {
    return prepareElevator(position, false);
  }

  /** Does not hold coral for alignment */
  public Command prepareElevatorAuto(ScoringPosition position) {
    return prepareElevator(position, true);
  }

  private Command scoringAction(ScoringPosition position, boolean isAuto) {
    final Command command;

    if (position.isObjectIntake()) {
      // Prepare for grabbing algae
      command = coralPlacer.grabAlgae();
    } else if (position.isAlgaeScore()) {
      // Score Algae
      Command delayCommand =
          Commands.sequence(
              Commands.waitUntil(hasObjectAssumeFalse.negate()), Commands.waitSeconds(1));

      command = delayCommand.deadlineFor(coralPlacer.expel());
    } else if (position.isTrough()) {
      Command delayCommand =
          Commands.sequence(
              Commands.waitUntil(hasObjectAssumeFalse.negate()), Commands.waitSeconds(0.5));

      command =
          Commands.sequence(
              delayCommand.deadlineFor(coralPlacer.expel()),
              elevator.setPosition(ElevatorPosition.troughClearPosition),
              coralPlacer
                  .setPosition(CoralPlacerPosition.grabPosition)
                  .until(coralPlacer.pastScorePosition),
              elevatorDown());
    } else {
      // Score coral
      command =
          Commands.sequence(
              coralPlacer.scoreAndExpel(),
              elevatorDown()
                  .onlyIf(
                      hasObjectAssumeFalse
                          .negate()
                          .and(() -> position != ScoringPosition.branchL4)));
    }

    return Commands.sequence(prepareElevator(position, isAuto), command);
  }

  public Command scoringAction(ScoringPosition position) {
    return scoringAction(position, false);
  }

  public Command scoringActionAuto(ScoringPosition position) {
    return scoringAction(position, true);
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command scoringActionOnTrigger(ScoringPosition position, BooleanSupplier scoreOrCancel) {
    // If the trigger returned true before the command exited normally, return instead of scoring
    return Commands.sequence(
        prepareElevator(position).until(scoreOrCancel),
        Commands.waitUntil(scoreOrCancel)
            .deadlineFor(coralPlacer.holdAlgae().onlyIf(position::isAlgae)),
        scoringAction(position)
            .onlyIf(() -> position.isTrough() || elevator.atPosition(position.elevatorPosition)));
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command selfCancellingScoringActionOnTrigger(
      Supplier<ScoringPosition> positionSupplier, BooleanSupplier scoreOrCancel) {
    return Commands.defer(
            () ->
                new ScheduleCommand(scoringActionOnTrigger(positionSupplier.get(), scoreOrCancel)),
            Set.of())
        .withInterruptBehavior(null);
  }

  public Command grabCoral() {
    return Commands.sequence(
        elevatorDown().until(() -> coralPlacer.atPosition(CoralPlacerPosition.grabPosition)),
        elevator.setPosition(ElevatorPosition.grabPosition),
        Commands.waitUntil(elevator.finishedGrab),
        Commands.waitUntil(hasObjectAssumeFalse).withTimeout(0.2),
        elevator.setPosition(ElevatorPosition.readyPosition),
        Commands.parallel(
                coralPlacer.setPosition(CoralPlacerPosition.holdPosition), coralPlacer.holdCoral())
            .onlyIf(hasObjectAssumeTrue));
  }

  public Command stowAlgae() {
    final CoralPlacerPosition coralPlacerGoal = CoralPlacerPosition.algaeStowPosition;
    final ElevatorPosition elevatorGoal = ElevatorPosition.readyPosition;

    return Commands.sequence(
        clearElevatorIfNeeded(elevatorGoal, coralPlacerGoal),
        coralPlacer.setPositionAlgae(coralPlacerGoal),
        elevator.setPosition(elevatorGoal));
  }

  public Command unstuckCoralPlacer() {
    return Commands.sequence(
        coralPlacer.setPosition(CoralPlacerPosition.unstuckPosition).withTimeout(0),
        elevator.setPosition(ElevatorPosition.grabPosition),
        coralPlacer.setPosition(CoralPlacerPosition.scorePosition));
  }

  public Command unstuckElevator(ElevatorPosition position) {
    return elevator.setPosition(position);
  }

  public Command prepForGrab() {
    return elevatorDown().unless(hasObjectAssumeTrue);
  }

  public Command jogCoralPlacerUp() {
    return coralPlacer.jog(CoralPlacerPosition.jogAmmount.unaryMinus());
  }

  public Command jogCoralPlacerDown() {
    return coralPlacer.jog(CoralPlacerPosition.jogAmmount);
  }
}
