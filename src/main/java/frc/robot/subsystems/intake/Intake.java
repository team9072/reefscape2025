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

  public Intake(PivotIO pivotIO, RollerIO rollerIO, RollerIO passthroughIO) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.passthroughIO = passthroughIO;

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
  }

  private void stopRollers() {
    rollerIO.stop();
    passthroughIO.stop();
  }

  private void intakeRollers() {
    rollerIO.setTorque(IntakeConstants.intakeTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);

    passthroughIO.setTorque(
        IntakeConstants.passthroughTorqueCurrent, IntakeConstants.passthroughTorqueDutyCycle);
  }

  private void reverseRollers() {
    rollerIO.setTorque(
        IntakeConstants.intakeReverseTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);

    passthroughIO.setTorque(
        IntakeConstants.passthroughReverseTorqueCurrent,
        IntakeConstants.passthroughTorqueDutyCycle);
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
    return Commands.sequence(
            runOnce(() -> pivotIO.setPosition(position.angle)),
            Commands.waitUntil(() -> position.withinTolerance(pivotInputs.position)))
        .finallyDo(pivotIO::setFloating);
  }
}
