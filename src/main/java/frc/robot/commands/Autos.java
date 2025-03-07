package frc.robot.commands;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.CoralFlow.ReefBranch;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import java.util.concurrent.atomic.AtomicBoolean;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Autos extends SubsystemBase {

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

  private final AutoFactory autoFactory;
  private final AutoInputs inputs;
  private final Robot.Subsystems s;

  public Autos(Robot.Subsystems s) {
    this.autoFactory =
        new AutoFactory(
            s.drive::getPose,
            (pose) -> {
              s.drive.setPose(pose);
              s.questNav.resetPose(pose);
            },
            s.drive::followTrajectory,
            true,
            s.drive);
    inputs = new AutoInputs(buildAutoChooser());
    this.s = s;
  }

  @Override
  public void periodic() {
    Logger.processInputs("Auto", inputs);
  }

  public Command getSelectedAuto() {
    return inputs.autoChooser.selectedCommand();
  }

  private void scorePreload(AutoTrajectory trajectory, Command afterScore) {
    trajectory
        .active()
        .onTrue(
            Commands.sequence(
                Commands.waitSeconds(0.5), s.coralFlow.prepareElevatorAuto(ReefBranch.branchL4)));
    trajectory
        .done()
        .onTrue(
            s.coralFlow
                .scoreCoral(ReefBranch.branchL4)
                .andThen(Commands.parallel(s.coralFlow.elevatorDown(), afterScore)));
  }

  private void intakeAndScore(
      AutoTrajectory intakeTrajectory, AutoTrajectory scoreTrajectory, Command afterScore) {
    final AtomicBoolean coralDetected = new AtomicBoolean(false);
    final AtomicBoolean coralGrabbed = new AtomicBoolean(false);

    intakeTrajectory
        .active()
        .onTrue(
            s.intake.setPosition(PivotPosition.deploy).withTimeout(0).andThen(s.intake.intake()));

    intakeTrajectory
        .active()
        .or(scoreTrajectory.active())
        .and(s.intake.coralDetected)
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> coralDetected.set(true)),
                Commands.waitSeconds(0.9),
                new ScheduleCommand(
                    Commands.parallel(
                        s.intake.setPosition(PivotPosition.stow),
                        Commands.sequence(
                            s.coralFlow.grabCoral(),
                            Commands.runOnce(() -> coralGrabbed.set(true)),
                            s.coralFlow.prepareElevatorAuto(ReefBranch.branchL4))))));

    intakeTrajectory.active().and(s.intake.coralDetected).onTrue(scoreTrajectory.cmd());
    intakeTrajectory.chain(scoreTrajectory);

    scoreTrajectory
        .done()
        .and(() -> !coralDetected.get())
        .onTrue(
            s.coralFlow
                .grabCoral()
                .andThen(
                    Commands.runOnce(
                            () -> {
                              coralDetected.set(true);
                              coralGrabbed.set(true);
                            })
                        .alongWith(s.intake.setPosition(PivotPosition.stow).withTimeout(0))));

    scoreTrajectory
        .recentlyDone()
        .and(coralGrabbed::get)
        .onTrue(
            s.coralFlow
                .scoreCoral(ReefBranch.branchL4)
                .andThen(Commands.parallel(s.coralFlow.elevatorDown(), afterScore)));
  }

  private AutoRoutine test() {
    AutoRoutine routine = autoFactory.newRoutine("C2S 1p");
    AutoTrajectory trajectory = routine.trajectory("C2S to R2P1 Bump");

    routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));

    scorePreload(trajectory, Commands.none());

    return routine;
  }

  private AutoRoutine test2p() {
    AutoRoutine routine = autoFactory.newRoutine("C2S 2p");
    AutoTrajectory scorePreloadTraj = routine.trajectory("C2S to R2P1 Bump");
    AutoTrajectory intakeS1Traj = routine.trajectory("R2P1 to S1 to R3P2", 0);
    AutoTrajectory scoreS1Traj = routine.trajectory("R2P1 to S1 to R3P2", 1);

    routine
        .active()
        .onTrue(Commands.sequence(scorePreloadTraj.resetOdometry(), scorePreloadTraj.cmd()));

    scorePreload(scorePreloadTraj, intakeS1Traj.cmd());
    intakeAndScore(intakeS1Traj, scoreS1Traj, Commands.none());

    return routine;
  }

  private AutoRoutine test3p() {
    AutoRoutine routine = autoFactory.newRoutine("C2S 3p");
    AutoTrajectory scorePreloadTraj = routine.trajectory("C2S to R2P1 Bump");

    AutoTrajectory intakeS1Traj = routine.trajectory("R2P1 to S1 to R3P2", 0);
    AutoTrajectory scoreS1Traj = routine.trajectory("R2P1 to S1 to R3P2", 1);

    AutoTrajectory intakeS2Traj = routine.trajectory("R3P2 to S2 to R4P1", 0);
    AutoTrajectory scoreS2Traj = routine.trajectory("R3P2 to S2 to R4P1", 1);

    routine
        .active()
        .onTrue(Commands.sequence(scorePreloadTraj.resetOdometry(), scorePreloadTraj.cmd()));

    scorePreloadTraj.active().onTrue(s.coralFlow.prepareElevatorAuto(ReefBranch.branchL4));

    scorePreload(scorePreloadTraj, intakeS1Traj.cmd());
    intakeAndScore(intakeS1Traj, scoreS1Traj, intakeS2Traj.cmd());
    intakeAndScore(intakeS2Traj, scoreS2Traj, Commands.none());

    return routine;
  }

  private AutoChooser buildAutoChooser() {
    AutoChooser autoChooser = new AutoChooser();

    autoChooser.addRoutine("C2S 1p", this::test);
    autoChooser.addRoutine("C2S 2p", this::test2p);
    autoChooser.addRoutine("C2S 3p", this::test3p);

    SmartDashboard.putData("Selected Auto", autoChooser);
    return autoChooser;
  }
}
