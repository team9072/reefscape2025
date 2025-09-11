package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.leds.Leds;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ScoringMemory {
  private ScoringPosition memorizedCoralPosition = ScoringPosition.branchL4;
  private ScoringPosition memorizedPosition;
  private boolean inAlgaeMode = false;

  private Leds leds;

  public ScoringMemory(Leds leds, ScoringPosition defaultPosition) {
    this.leds = leds;

    memorizePositionInternal(defaultPosition);
  }

  public ScoringPosition getMemorizedPosition() {
    return memorizedPosition;
  }

  public boolean inAlgaeMode() {
    return inAlgaeMode;
  }

  public Command setAlgaeMode(boolean algaeMode) {
    return Commands.runOnce(
        () -> {
          inAlgaeMode = algaeMode;
          updateLedAnimation();
        });
  }

  public Command toggleAlgaeMode() {
    return Commands.runOnce(
        () -> {
          inAlgaeMode = !inAlgaeMode;
          updateLedAnimation();
        });
  }

  private void updateLedAnimation() {
    if (inAlgaeMode) {
      leds.setAnimation(ScoringAnimations.algae);
      return;
    }

    switch (memorizedPosition) {
      case troughL1:
        leds.setAnimation(ScoringAnimations.troughL1);
        break;

      case branchL2:
        leds.setAnimation(ScoringAnimations.branchL2);
        break;

      case branchL3:
        leds.setAnimation(ScoringAnimations.branchL3);
        break;

      case branchL4:
        leds.setAnimation(ScoringAnimations.branchL4);
        break;

      default:
        leds.clearAnimation();
        break;
    }
  }

  private void memorizePositionInternal(ScoringPosition position) {
    memorizedPosition = position;
    if (position.isBranch() || position.isTrough()) {
      memorizedCoralPosition = position;
    }

    updateLedAnimation();

    Logger.recordOutput("CoralFlow/MemorizedPosition", memorizedPosition.name());
    Logger.recordOutput("CoralFlow/MemorizedCoralPosition", memorizedCoralPosition.name());
  }

  public Command memorizePosition(ScoringPosition position) {
    return Commands.runOnce(() -> memorizePositionInternal(position));
  }

  public Command memorizeEither(
      ScoringPosition onFalse, ScoringPosition onTrue, BooleanSupplier selector) {
    return Commands.runOnce(
        () -> memorizePositionInternal(selector.getAsBoolean() ? onTrue : onFalse));
  }

  public Command memorizeCoralAlgae(ScoringPosition ifCoralMode, ScoringPosition ifAlgaeMode) {
    return memorizeEither(ifCoralMode, ifAlgaeMode, this::inAlgaeMode);
  }

  public Command restoreCoralPosition() {
    return Commands.runOnce(() -> memorizePositionInternal(memorizedCoralPosition))
        .alongWith(setAlgaeMode(false));
  }
}
