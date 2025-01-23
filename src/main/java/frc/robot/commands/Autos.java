package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Autos extends SubsystemBase {
  private final AutoFactory autoFactory;
  private final AutoInputs inputs;

  private static class AutoInputs implements LoggableInputs {
    final AutoChooser autoChooser;

    public AutoInputs(AutoChooser autoChooser) {
      this.autoChooser = autoChooser;
    }

    @Override
    public void toLog(LogTable table) {
      String name = autoChooser.selectedCommand().getName();

      if (name == "InstantCommand") name = "Nothing";

      table.put("Selected", name);
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

    autoChooser.addRoutine("Forward 180", this::forward180Auto);
    autoChooser.addRoutine("Around Reef", this::aroundReefAuto);

    SmartDashboard.putData("Selected Auto", autoChooser);
    return autoChooser;
  }

  private AutoRoutine forward180Auto() {
    AutoRoutine routine = autoFactory.newRoutine("Forward 180");
    AutoTrajectory trajectory = routine.trajectory("Forward 180");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }

  private AutoRoutine aroundReefAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Around Reef");
    AutoTrajectory trajectory = routine.trajectory("Around Reef");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    return routine;
  }
}
