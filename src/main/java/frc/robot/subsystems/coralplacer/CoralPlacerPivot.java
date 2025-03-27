package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.generic.roller.RollerIO;
import org.littletonrobotics.junction.Logger;

public class CoralPlacerPivot extends SubsystemBase {
  private final CoralPlacerIO coralPlacerIO;
  private final CoralPlacerInputsAutoLogged coralPlacerInputs = new CoralPlacerInputsAutoLogged();

  private MutAngle jogOffset = Rotations.mutable(0);
  private CoralPlacerPosition lastPosition = CoralPlacerPosition.stowPosition;

  public final Trigger pastScorePosition =
      new Trigger(
          () ->
              coralPlacerInputs.position.gt(
                  getModifiedAngle(CoralPlacerPosition.scoreCompleteThreshold)));

  public final Trigger clearsReef =
      new Trigger(
          () ->
              atPosition(CoralPlacerPosition.scorePosition)
                  || coralPlacerInputs.position.lt(
                      getModifiedAngle(CoralPlacerPosition.scorePosition.angle)));

  public CoralPlacerPivot(CoralPlacerIO coralPlacerIO, RollerIO rollerIO) {
    this.coralPlacerIO = coralPlacerIO;
  }

  @Override
  public void periodic() {
    coralPlacerIO.updateInputs(coralPlacerInputs);
    Logger.processInputs("Coral Placer/Pivot", coralPlacerInputs);
  }

  private Angle getModifiedAngle(Angle angle) {
    return angle.plus(jogOffset);
  }

  private boolean atPosition(CoralPlacerPosition position) {
    return getModifiedAngle(position.angle).isNear(coralPlacerInputs.position, Degrees.of(2))
        && coralPlacerInputs.velocity.isNear(
            RotationsPerSecond.zero(), CoralPlacerPosition.velocityTolerance);
  }

  private void setPositionWithJog() {
    if (lastPosition == CoralPlacerPosition.removeAlgaePosition) {
      coralPlacerIO.setPositionAlgae(getModifiedAngle(lastPosition.angle));
    } else {
      coralPlacerIO.setPosition(getModifiedAngle(lastPosition.angle));
    }
  }

  private void setPositionWithJog(CoralPlacerPosition position) {
    lastPosition = position;
    setPositionWithJog();
  }

  public Command setPosition(CoralPlacerPosition position) {
    return run(() -> {
          jogOffset.mut_replace(Rotations.zero());
          setPositionWithJog(position);
        })
        .until(() -> atPosition(position));
  }

  public Command jog(Angle offsetAngle) {
    return runOnce(
        () -> {
          jogOffset.mut_plus(offsetAngle);
          setPositionWithJog();
        });
  }
}
