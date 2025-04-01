package frc.robot.commands.utils;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;

public final class RumbleCommands {
  public static record Rumble(RumbleType type, double value) {
    public Rumble(double value) {
      this(RumbleType.kBothRumble, value);
    }
  }

  public static Command rumble(CommandGenericHID controller, Rumble rumble) {
    return Commands.startEnd(
        () -> controller.setRumble(rumble.type(), rumble.value()),
        () -> controller.setRumble(rumble.type(), 0));
  }
}
