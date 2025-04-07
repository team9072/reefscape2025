package frc.robot.commands.scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ScoringMemory {
  private ScoringPosition memorizedCoralPosition = ScoringPosition.branchL4;
  private ScoringPosition memorizedPosition;

  public ScoringMemory(ScoringPosition defaultPosition) {
    memorizePositionInternal(defaultPosition);
  }

  public ScoringPosition getMemorizedPosition() {
    return memorizedPosition;
  }

  private void memorizePositionInternal(ScoringPosition position) {
    memorizedPosition = position;
    if (position.isBranch() || position.isTrough()) {
      memorizedCoralPosition = position;
    }

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

  public Command restoreCoralPosition() {
    return Commands.defer(() -> memorizePosition(memorizedCoralPosition), Set.of());
  }
}
