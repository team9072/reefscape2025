package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import org.littletonrobotics.junction.Logger;

public class CoralPlacer extends SubsystemBase {
  private final CoralPlacerIO coralPlacerIO;
  private final CoralPlacerInputsAutoLogged coralPlacerInputs = new CoralPlacerInputsAutoLogged();

  public final Trigger pastScorePosition =
      new Trigger(() -> coralPlacerInputs.position.gt(CoralPlacerPosition.scoreCompleteThreshold));

  public final Trigger clearsReef =
      new Trigger(
          () ->
              atPosition(CoralPlacerPosition.scorePosition)
                  || coralPlacerInputs.position.lt(CoralPlacerPosition.scorePosition.angle));

  public CoralPlacer(CoralPlacerIO coralPlacerIO) {
    this.coralPlacerIO = coralPlacerIO;
  }

  @Override
  public void periodic() {
    coralPlacerIO.updateInputs(coralPlacerInputs);
    Logger.processInputs("Coral Placer", coralPlacerInputs);
  }

  private boolean atPosition(CoralPlacerPosition position) {
    return position.angle.isNear(coralPlacerInputs.position, Degrees.of(2))
        && coralPlacerInputs.velocity.isNear(
            RotationsPerSecond.zero(), CoralPlacerPosition.velocityTolerance);
  }

  public Command setPosition(CoralPlacerPosition position) {
    return run(() -> {
          if (position == CoralPlacerPosition.removeAlgaePosition) {
            coralPlacerIO.setPositionAlgae(position.angle);
          } else {
            coralPlacerIO.setPosition(position.angle);
          }
        })
        .until(() -> atPosition(position));
  }
}
