package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.climber.ClimberConstants.ClimberPosition;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import java.util.function.DoubleSupplier;

public final class ClimbCommands {
  public static Command enterClimbMode(Robot.Subsystems s) {
    return Commands.parallel(
        s.scoring.elevatorDown(),
        s.intake.setPosition(PivotPosition.deploy),
        s.climber.setPosition(ClimberPosition.deployPosition));
  }

  public static Command manualControl(Robot.Subsystems s, DoubleSupplier outputSupplier) {
    return Commands.sequence(
        // enterClimbMode(s),
        s.climber.manualControl(outputSupplier));
  }
}
