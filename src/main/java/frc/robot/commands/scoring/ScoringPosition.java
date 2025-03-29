package frc.robot.commands.scoring;

import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorPosition;

public enum ScoringPosition {
  troughL1(ElevatorPosition.reefTroughPosition),

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
