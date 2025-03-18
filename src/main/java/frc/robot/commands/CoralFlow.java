package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coralplacer.CoralPlacer;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;
import frc.robot.subsystems.intake.Intake;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class CoralFlow {
  public enum ReefBranch {
    branchL2(ElevatorPosition.reefL2Position),
    branchL3(ElevatorPosition.reefL3Position),
    branchL4(ElevatorPosition.reefL4Position);

    public ElevatorPosition position;

    ReefBranch(ElevatorPosition position) {
      this.position = position;
    }
  }

  private final Intake intake;
  private final Elevator elevator;
  private final CoralPlacer coralPlacer;

  // TODO: account for pre-load
  private boolean hasCoral = false;
  private ReefBranch memorizedBranch = ReefBranch.branchL4;

  public CoralFlow(Intake intake, Elevator elevator, CoralPlacer coralPlacer) {
    this.intake = intake;
    this.elevator = elevator;
    this.coralPlacer = coralPlacer;
  }

  public ReefBranch getMemorizedBranch() {
    return memorizedBranch;
  }

  public Command memorizeBranch(ReefBranch branch) {
    return Commands.runOnce(() -> memorizedBranch = branch);
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

  public Command scoreCoral(ReefBranch branch, boolean dontHold) {
    return Commands.sequence(
        prepareElevator(branch, dontHold),
        coralPlacer
            .setPosition(CoralPlacerPosition.scorePosition)
            .until(coralPlacer.pastScorePosition));
  }

  public Command scoreCoral(ReefBranch branch) {
    return scoreCoral(branch, false);
  }

  public Command scoreCoralAuto(ReefBranch branch) {
    return scoreCoral(branch, true);
  }

  public Command prepareElevator(ReefBranch branch, boolean dontHold) {
    return elevator
        .clearCoral()
        .andThen(
            coralPlacer
                .setPosition(CoralPlacerPosition.holdPosition)
                .unless(() -> elevator.atPosition(branch.position)),
            elevator.setPosition(branch.position),
            coralPlacer
                .setPosition(CoralPlacerPosition.preScoreHoldPosition)
                .unless(() -> dontHold));
  }

  public Command prepareElevator(ReefBranch branch) {
    return prepareElevator(branch, false);
  }

  /** Does not hold coral for alignment */
  public Command prepareElevatorAuto(ReefBranch branch) {
    return prepareElevator(branch, true);
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command scoreCoralOnTrigger(ReefBranch branch, BooleanSupplier scoreOrCancel) {
    // If the trigger returned true before the command exited normally, return instead of scoring
    return Commands.sequence(
        prepareElevator(branch).until(scoreOrCancel),
        Commands.waitUntil(scoreOrCancel),
        coralPlacer
            .setPosition(CoralPlacerPosition.scorePosition)
            .onlyIf(() -> elevator.atPosition(branch.position)));
  }

  /**
   * Creates a command to raise up the elevator (prepare) and then score the coral. Once the
   * `scoreOrCancel` supplier returns true, the robot will score the coral if it is ready.
   */
  public Command scoreCoralOnTrigger(Supplier<ReefBranch> branch, BooleanSupplier scoreOrCancel) {
    return Commands.defer(
        () -> scoreCoralOnTrigger(branch.get(), scoreOrCancel), Set.of(elevator, coralPlacer));
  }

  public Command grabCoral() {
    return Commands.sequence(
        Commands.parallel(
            coralPlacer.setPosition(CoralPlacerPosition.grabPosition),
            elevator.setPosition(ElevatorPosition.readyPosition).withTimeout(0)),
        elevator.setPosition(ElevatorPosition.grabPosition),
        Commands.waitSeconds(0.2),
        elevator.setPosition(ElevatorPosition.readyPosition),
        coralPlacer.setPosition(CoralPlacerPosition.holdPosition));
  }
}
