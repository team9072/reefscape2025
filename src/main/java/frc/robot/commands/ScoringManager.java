package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class ScoringManager {
  public enum ScoringPosition {
    troughL1(null),

    branchL2(ElevatorPosition.reefL2Position),
    branchL3(ElevatorPosition.reefL3Position),
    branchL4(ElevatorPosition.reefL4Position),

    coralGround(ElevatorPosition.groundPickupPosition),
    algaeGround(ElevatorPosition.groundPickupPosition),
    algaeL2(ElevatorPosition.algaeL2Position),
    algaeL3(ElevatorPosition.algaeL3Position),

    processor(ElevatorPosition.groundPickupPosition),
    barge(ElevatorPosition.reefL4Position);

    public final ElevatorPosition elevatorPosition;

    ScoringPosition(ElevatorPosition position) {
      this.elevatorPosition = position;
    }

    public boolean isObjectIntake() {
      switch (this) {
        case coralGround, algaeGround, algaeL2, algaeL3:
          return true;
        default:
          return false;
      }
    }

    public boolean isAlgaeScore() {
      switch (this) {
        case processor, barge:
          return true;
        default:
          return false;
      }
    }

    public boolean isTrough() {
      switch (this) {
        case troughL1:
          return true;
        default:
          return false;
      }
    }

    public boolean isBranch() {
      switch (this) {
        case branchL2, branchL3, branchL4:
          return true;
        default:
          return false;
      }
    }
  }

  private final Elevator elevator;
  private final CoralPlacer coralPlacer;

  private ScoringPosition memorizedBranchPosition = ScoringPosition.branchL4;
  private ScoringPosition memorizedPosition = memorizedBranchPosition;

  public ScoringManager(Elevator elevator, CoralPlacer coralPlacer) {
    this.elevator = elevator;
    this.coralPlacer = coralPlacer;
  }

  public ScoringPosition getMemorizedPosition() {
    return memorizedPosition;
  }

  public Command memorizePosition(ScoringPosition position) {
    return Commands.runOnce(
        () -> {
          memorizedPosition = position;
          if (position.isBranch()) {
            memorizedBranchPosition = position;
          }

          Logger.recordOutput("CoralFlow/MemorizedPosition", memorizedPosition.name());
          Logger.recordOutput("CoralFlow/MemorizedBranch", memorizedBranchPosition.name());
        });
  }

  /** Returns the elevator to the ready position, and prepares the coral placer for a grab */
  public Command elevatorDown() {
    return elevator
        .setPosition(ElevatorPosition.readyPosition)
        .alongWith(coralPlacer.setPosition(CoralPlacerPosition.grabPosition));
  }

  /**
   * Returns the elevator to the ready position. The command only runs until the elevator is clear
   * of the staging zone
   */
  public Command clearElevator() {
    return elevator.clearCoral();
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
          Commands.sequence(
              coralPlacer.setPosition(coralPlacerGoal),
              elevator.setPosition(position.elevatorPosition));
    } else if (position.isAlgaeScore()) {
      // Prepare to score algae
      final boolean isBarge = position == ScoringPosition.barge;
      coralPlacerGoal =
          isBarge
              ? CoralPlacerPosition.scoreBargePosition
              : CoralPlacerPosition.scoreProcessorPosition;

      command =
          Commands.sequence(
              coralPlacer.setPosition(CoralPlacerPosition.holdPosition).onlyIf(() -> isBarge),
              elevator.setPosition(position.elevatorPosition),
              coralPlacer.setPosition(coralPlacerGoal));
    } else {
      // Score coral
      final boolean isL4 = position == ScoringPosition.branchL4;

      coralPlacerGoal =
          isAuto
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

    return Commands.sequence(elevator.clearCoral(), command)
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
      command = coralPlacer.grabObject();
    } else if (position.isAlgaeScore()) {
      // Score Algae
      Command delayCommand =
          Commands.sequence(
              Commands.waitUntil(coralPlacer.hasObject.negate()), Commands.waitSeconds(1));

      command = delayCommand.deadlineFor(coralPlacer.expel());
    } else {
      // Score coral
      command =
          Commands.sequence(
              coralPlacer.scoreAndExpel(),
              elevatorDown()
                  .onlyIf(
                      coralPlacer
                          .hasObject
                          .negate()
                          .and(() -> (position != ScoringPosition.branchL4))));
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
        Commands.waitUntil(scoreOrCancel),
        scoringAction(position)
            .onlyIf(() -> position.isTrough() || elevator.atPosition(position.elevatorPosition)));
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command scoringActionOnTrigger(
      Supplier<ScoringPosition> positionSupplier, BooleanSupplier scoreOrCancel) {
    return Commands.defer(
        () -> scoringActionOnTrigger(positionSupplier.get(), scoreOrCancel),
        Set.of(elevator, coralPlacer.pivot(), coralPlacer.rollers()));
  }

  public Command grabCoral() {
    return Commands.sequence(
        Commands.parallel(
            Commands.defer(() -> memorizePosition(memorizedBranchPosition), Set.of()),
            coralPlacer.setPosition(CoralPlacerPosition.grabPosition),
            elevator.setPosition(ElevatorPosition.readyPosition).withTimeout(0)),
        elevator.setPosition(ElevatorPosition.grabPosition),
        Commands.waitUntil(coralPlacer.hasObject).withTimeout(0.2),
        elevator.setPosition(ElevatorPosition.readyPosition),
        coralPlacer.setPosition(CoralPlacerPosition.holdPosition).onlyIf(coralPlacer.hasObject));
  }

  public Command unstuckCoralPlacer() {
    return Commands.sequence(
        coralPlacer.setPosition(CoralPlacerPosition.unstuckPosition).withTimeout(0),
        elevator.setPosition(ElevatorPosition.grabPosition),
        coralPlacer.setPosition(CoralPlacerPosition.scorePosition));
  }

  public Command coralPlacerForwardAuto() {
    return coralPlacer.setPosition(CoralPlacerPosition.scorePosition);
  }

  public Command unstuckElevator(ElevatorPosition position) {
    return elevator.setPosition(position);
  }

  public Command jogCoralPlacerUp() {
    return coralPlacer.jog(CoralPlacerPosition.jogAmmount.unaryMinus());
  }

  public Command jogCoralPlacerDown() {
    return coralPlacer.jog(CoralPlacerPosition.jogAmmount);
  }
}
