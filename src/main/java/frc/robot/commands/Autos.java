package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Autos extends SubsystemBase {
  private final AutoFactory autoFactory;
  private final AutoInputs inputs;

  private class AutoInputs implements LoggableInputs {
    final AutoChooser autoChooser;

    public AutoInputs(AutoChooser autoChooser) {
      this.autoChooser = autoChooser;
    }

    @Override
    public void toLog(LogTable table) {
      table.put("Selected", autoChooser.selectedCommand().getName());
    }

    @Override
    public void fromLog(LogTable table) {
      autoChooser.select(table.get("Selected", "Nothing"));
    }
  }

  public Autos(Drive drive) {
    this.autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
    inputs = new AutoInputs(buildAutoChooser());
  }

  @Override
  public void periodic() {
    Logger.processInputs("Auto", inputs);
  }

  public Command getSelectedAuto() {
    return inputs.autoChooser.selectedCommand();
  }

  private AutoChooser buildAutoChooser() {
    AutoChooser autoChooser = new AutoChooser();
    return autoChooser;
  }
}
