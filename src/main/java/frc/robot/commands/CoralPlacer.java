package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.coralplacer.CoralPlacerPivot;
import frc.robot.subsystems.coralplacer.CoralPlacerRollers;

public class CoralPlacer {
  private final CoralPlacerPivot pivot;
  private final CoralPlacerRollers rollers;

  public final Trigger pastScorePosition;
  public final Trigger clearsReef;

  public final Trigger hasObject;

  public CoralPlacer(CoralPlacerPivot pivot, CoralPlacerRollers rollers) {
    this.pivot = pivot;
    pastScorePosition = pivot.pastScorePosition;
    clearsReef = pivot.clearsReef;

    this.rollers = rollers;
    this.hasObject = rollers.hasObject;

    rollers.setDefaultCommand(
        Commands.sequence(
                rollers.neutral().until(hasObject),
                rollers.grab().until(hasObject.negate().debounce(1)))
            .repeatedly());
  }

  public Subsystem pivot() {
    return pivot;
  }

  public Subsystem rollers() {
    return rollers;
  }

  public Command setPosition(CoralPlacerPosition position) {
    return pivot.setPosition(position);
  }

  public Command jog(Angle offsetAngle) {
    return pivot.jog(offsetAngle);
  }

  public Command grabObject() {
    return rollers.grab().until(hasObject);
  }

  public Command expel() {
    return rollers.reverse();
  }

  public Command scoreAndExpel() {
    return Commands.parallel(
            rollers.neutral(),
            pivot.setPosition(CoralPlacerPosition.scorePosition),
            Commands.sequence(Commands.waitSeconds(0.2), Commands.waitUntil(pivot.atLowVelocity)))
        .andThen(rollers.reverse())
        .until(pastScorePosition);
  }
}
