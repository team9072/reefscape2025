package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  private final RollerIO passthroughIO;
  private final RollerIOInputsAutoLogged passthroughInputs = new RollerIOInputsAutoLogged();

  private final RollerIO stagingIO;
  private final RollerIOInputsAutoLogged stagingInputs = new RollerIOInputsAutoLogged();

  private PivotPosition lastSetpoint = null;

  public Intake(PivotIO pivotIO, RollerIO rollerIO, RollerIO passthroughIO, RollerIO stagingIO) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.passthroughIO = passthroughIO;
    this.stagingIO = stagingIO;

    pivotIO.setFloating();
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);

    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    passthroughIO.updateInputs(passthroughInputs);
    Logger.processInputs("Intake/Passthrough", passthroughInputs);

    stagingIO.updateInputs(stagingInputs);
    Logger.processInputs("Intake/Staging", stagingInputs);
  }

  private void stopRollers() {
    rollerIO.stop();
    passthroughIO.stop();
    stagingIO.stop();
  }

  private void intakeRollers() {
    rollerIO.setTorque(IntakeConstants.intakeTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);

    passthroughIO.setTorque(
        IntakeConstants.passthroughTorqueCurrent, IntakeConstants.passthroughTorqueDutyCycle);

    stagingIO.setTorque(
        IntakeConstants.stagingTorqueCurrent, IntakeConstants.stagingTorqueDutyCycle);
  }

  private void reverseRollers() {
    rollerIO.setTorque(
        IntakeConstants.intakeReverseTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);

    passthroughIO.setTorque(
        IntakeConstants.passthroughReverseTorqueCurrent,
        IntakeConstants.passthroughTorqueDutyCycle);

    stagingIO.setTorque(
        IntakeConstants.stagingReverseTorqueCurrent, IntakeConstants.stagingTorqueDutyCycle);
  }

  public Command intake() {
    return runEnd(this::intakeRollers, this::stopRollers);
  }

  public Command intakeAlgae() {
    return runEnd(
        () ->
            rollerIO.setTorque(
                IntakeConstants.intakeAlgaeTorqueCurrent,
                IntakeConstants.intakeAlgaeTorqueDutyCycle),
        this::stopRollers);
  }

  public Command reverse() {
    return runEnd(this::reverseRollers, this::stopRollers);
  }

  public Command setPosition(PivotPosition position) {
    Command command =
        Commands.sequence(
            runOnce(
                () -> {
                  pivotIO.setPosition(position.angle);
                  lastSetpoint = position;
                }),
            Commands.waitUntil(() -> position.withinTolerance(pivotInputs.position)));

    if (position.shouldFloat) {
      command = command.finallyDo(pivotIO::setFloating);
    }

    return command;
  }

  public Command toggleDeploy() {
    return Commands.either(
        setPosition(PivotPosition.stow),
        setPosition(PivotPosition.deploy),
        () -> {
          if (lastSetpoint == null) {
            return true;
          }

          return (lastSetpoint != PivotPosition.stow);
        });
  }
}
