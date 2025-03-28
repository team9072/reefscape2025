package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class CoralFlow {
  public enum ReefPosition {
    troughL1(null),

    branchL2(ElevatorPosition.reefL2Position),
    branchL3(ElevatorPosition.reefL3Position),
    branchL4(ElevatorPosition.reefL4Position),

    algaeL2(ElevatorPosition.algaeL2Position),
    algaeL3(ElevatorPosition.algaeL3Position);

    public final ElevatorPosition elevatorPosition;

    ReefPosition(ElevatorPosition position) {
      this.elevatorPosition = position;
    }

    public boolean isAlgae() {
      switch (this) {
        case algaeL2, algaeL3:
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

  private final Intake intake;
  private final Elevator elevator;
  private final CoralPlacer coralPlacer;

  private ReefPosition memorizedBranchPosition = ReefPosition.branchL4;
  private ReefPosition memorizedPosition = memorizedBranchPosition;

  public CoralFlow(Intake intake, Elevator elevator, CoralPlacer coralPlacer) {
    this.intake = intake;
    this.elevator = elevator;
    this.coralPlacer = coralPlacer;
  }

  public ReefPosition getMemorizedPosition() {
    return memorizedPosition;
  }

  public Command memorizePosition(ReefPosition position) {
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

  private Command reefAction(ReefPosition position, boolean isAuto) {
    if (position.isAlgae()) {
      // Prepare for grabbing algae
      return Commands.sequence(prepareElevator(position, isAuto), coralPlacer.grabObject());
    } else {
      // Score coral
      return Commands.sequence(
          prepareElevator(position, isAuto)
              .unless(() -> elevator.atPosition(position.elevatorPosition)),
          coralPlacer.scoreAndExpel());
    }
  }

  public Command reefAction(ReefPosition position) {
    return reefAction(position, false);
  }

  public Command reefActionAuto(ReefPosition position) {
    return reefAction(position, true);
  }

  private Command prepareElevator(ReefPosition position, boolean isAuto) {
    return Commands.sequence(
        elevator.clearCoral(),
        coralPlacer
            .setPosition(
                position.isAlgae()
                    ? CoralPlacerPosition.grabAlgaePosition
                    : CoralPlacerPosition.holdPosition)
            .unless(() -> elevator.atPosition(position.elevatorPosition) && position.isBranch()),
        elevator.setPosition(position.elevatorPosition),
        coralPlacer
            .setPosition(
                position == ReefPosition.branchL4
                    ? CoralPlacerPosition.preScoreHoldPositionL4
                    : CoralPlacerPosition.preScoreHoldPosition)
            .unless(() -> isAuto || !position.isBranch()));
  }

  public Command prepareElevator(ReefPosition position) {
    return prepareElevator(position, false);
  }

  /** Does not hold coral for alignment */
  public Command prepareElevatorAuto(ReefPosition position) {
    return prepareElevator(position, true);
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command reefActionOnTrigger(ReefPosition position, BooleanSupplier scoreOrCancel) {
    // If the trigger returned true before the command exited normally, return instead of scoring
    return Commands.sequence(
        prepareElevator(position).until(scoreOrCancel),
        Commands.waitUntil(scoreOrCancel),
        reefAction(position)
            .onlyIf(() -> position.isTrough() || elevator.atPosition(position.elevatorPosition)));
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command reefActionOnTrigger(
      Supplier<ReefPosition> positionSupplier, BooleanSupplier scoreOrCancel) {
    return Commands.defer(
        () -> reefActionOnTrigger(positionSupplier.get(), scoreOrCancel),
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
        coralPlacer.setPosition(CoralPlacerPosition.holdPosition));
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

  public Command scoreIntoBargeOnTrigger(Trigger scoreOrCancel) {
    Command prepare =
        Commands.sequence(
            coralPlacer.setPosition(CoralPlacerPosition.scoreAlgaePosition),
            elevator.setPosition(ElevatorPosition.reefL4Position));

    return Commands.sequence(
        prepare.until(scoreOrCancel),
        Commands.waitUntil(scoreOrCancel),
        coralPlacer.expel().withTimeout(2));
  }
}
