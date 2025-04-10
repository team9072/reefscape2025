package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.coralplacer.CoralPlacerPivot;
import frc.robot.subsystems.coralplacer.CoralPlacerRollers;
import java.util.function.BooleanSupplier;

public class CoralPlacer {
  private final CoralPlacerPivot pivot;
  private final CoralPlacerRollers rollers;

  public final Trigger pastScorePosition;
  public final Trigger clearsReef;
  public final Trigger clearsTop;

  public final Trigger hasObject;

  public CoralPlacer(CoralPlacerPivot pivot, CoralPlacerRollers rollers) {
    this.pivot = pivot;
    pastScorePosition = pivot.pastScorePosition;
    clearsReef = pivot.clearsReef;
    clearsTop = pivot.clearsTop;

    this.rollers = rollers;
    this.hasObject = rollers.hasObject;
  }

  public Subsystem pivot() {
    return pivot;
  }

  public Subsystem rollers() {
    return rollers;
  }

  public boolean atPosition(CoralPlacerPosition position) {
    return pivot.atPosition(position);
  }

  public Command setPosition(CoralPlacerPosition position) {
    return pivot.setPosition(position);
  }

  public Command setPositionAlgae(CoralPlacerPosition position) {
    return pivot.setPositionAlgae(position);
  }

  public Command jog(Angle offsetAngle) {
    return pivot.jog(offsetAngle);
  }

  public Command grabObject() {
    return Commands.sequence(Commands.waitUntil(hasObject), Commands.waitSeconds(1))
        .deadlineFor(rollers.grab());
  }

  public Command hold() {
    return rollers.hold();
  }

  public Command holdWhile(BooleanSupplier shouldGrab) {
    return Commands.sequence(
            Commands.waitUntil(shouldGrab),
            rollers.hold().until(new Trigger(shouldGrab).negate().debounce(1)))
        .repeatedly();
  }

  public Command expel() {
    return rollers.reverse();
  }

  public Command scoreAndExpel() {
    return Commands.parallel(
            pivot.setPosition(CoralPlacerPosition.scorePosition),
            Commands.sequence(Commands.waitSeconds(0.2), Commands.waitUntil(pivot.atLowVelocity)))
        .andThen(rollers.reverse())
        .until(pastScorePosition);
  }

  public Command clearTop() {
    return setPosition(CoralPlacerPosition.holdPosition).until(clearsTop).unless(clearsTop);
  }

  private boolean wouldCrossZone(CoralPlacerPosition goal, Angle maxAngle, Angle minAngle) {
    boolean currentlyInZone = pivot.position().lt(maxAngle) && pivot.position().gt(minAngle);
    if (currentlyInZone) {
      return true;
    }

    boolean goalInZone = goal.angle.lt(maxAngle) && goal.angle.gt(minAngle);
    if (goalInZone) {
      return true;
    }

    if (pivot.position().lt(minAngle) && goal.angle.gt(maxAngle)) {
      return true;
    }

    if (pivot.position().gt(maxAngle) && goal.angle.lt(minAngle)) {
      return true;
    }

    return false;
  }

  public boolean wouldCrossCupHitZone(CoralPlacerPosition goal) {
    return wouldCrossZone(
        goal, CoralPlacerConstants.cupHitMaxAngle, CoralPlacerConstants.cupHitMinAngle);
  }

  public boolean wouldCrossTopHitZone(CoralPlacerPosition goal) {
    return wouldCrossZone(
        goal, CoralPlacerConstants.topHitMaxAngle, CoralPlacerConstants.topHitMinAngle);
  }
}
