package frc.robot.commands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.coralplacer.CoralPlacerConstants.CoralPlacerPosition;
import frc.robot.subsystems.coralplacer.CoralPlacerPivot;
import frc.robot.subsystems.coralplacer.CoralPlacerRollers;

public class CoralPlacer {
  private final CoralPlacerPivot pivot;
  private final CoralPlacerRollers rollers;

  public final Trigger pastScorePosition;
  public final Trigger clearsReef;

  public CoralPlacer(CoralPlacerPivot pivot, CoralPlacerRollers rollers) {
    this.pivot = pivot;
    pastScorePosition = pivot.pastScorePosition;
    clearsReef = pivot.clearsReef;
    this.rollers = rollers;
  }

  public Command setPosition(CoralPlacerPosition position) {
    return pivot.setPosition(position);
  }

  public Command jog(Angle offsetAngle) {
    return pivot.jog(offsetAngle);
  }

  public Command rollersGrab() {
    return rollers.grab();
  }
}
