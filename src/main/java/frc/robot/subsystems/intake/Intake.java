package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.generic.beambreak.BeamBreakIO;
import frc.robot.subsystems.generic.beambreak.BeamBreakIOInputsAutoLogged;
import frc.robot.subsystems.generic.roller.RollerIO;
import frc.robot.subsystems.generic.roller.RollerIOInputsAutoLogged;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final PivotIO pivotIO;
  private final PivotIOInputsAutoLogged pivotInputs = new PivotIOInputsAutoLogged();

  private final RollerIO rollerIO;
  private final RollerIOInputsAutoLogged rollerInputs = new RollerIOInputsAutoLogged();

  private final RollerIO passthroughIO;
  private final RollerIOInputsAutoLogged passthroughInputs = new RollerIOInputsAutoLogged();

  private final BeamBreakIO passthroughBeamBreakIO;
  private final BeamBreakIOInputsAutoLogged passthroughBeamBreakInputs =
      new BeamBreakIOInputsAutoLogged();

  private final BeamBreakIO stagingBeamBreakIO;
  private final BeamBreakIOInputsAutoLogged stagingBeamBreakInputs =
      new BeamBreakIOInputsAutoLogged();

  private PivotPosition lastSetpoint = PivotPosition.stow;
  private Trigger isAtZeroVelocity =
      new Trigger(() -> pivotInputs.velocity.isEquivalent(RotationsPerSecond.zero())).debounce(0.2);

  private boolean homedSinceLastSetpoint = false;
  private Timer waitForHoming = new Timer();
  private Timer homingDelay = new Timer();

  public Trigger coralInPassthrough = new Trigger(() -> false);
  public Trigger coralStaged = new Trigger(() -> stagingBeamBreakInputs.objectDetected);

  public Intake(
      PivotIO pivotIO,
      RollerIO rollerIO,
      RollerIO passthroughIO,
      BeamBreakIO passthroughBeamBreakIO,
      BeamBreakIO stagingBeamBreakIO) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.passthroughIO = passthroughIO;
    this.passthroughBeamBreakIO = passthroughBeamBreakIO;
    this.stagingBeamBreakIO = stagingBeamBreakIO;
  }

  @Override
  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
    Logger.processInputs("Intake/Pivot", pivotInputs);

    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    passthroughIO.updateInputs(passthroughInputs);
    Logger.processInputs("Intake/Passthrough", passthroughInputs);

    passthroughBeamBreakIO.updateInputs(passthroughBeamBreakInputs);
    Logger.processInputs("Intake/Passthrough Beam Break", passthroughBeamBreakInputs);

    stagingBeamBreakIO.updateInputs(stagingBeamBreakInputs);
    Logger.processInputs("Intake/Staging Beam Break", stagingBeamBreakInputs);

    if (lastSetpoint == PivotPosition.stow && !homedSinceLastSetpoint && !isHoming()) {
      startWaitForHoming();
    }

    // Wait for reach position or time elapsed
    if (waitForHoming.isRunning()
        && (waitForHoming.hasElapsed(0.5) || lastSetpoint.withinTolerance(pivotInputs.position))) {
      startHoming();
    }

    if (homingDelay.hasElapsed(0.2) && isAtZeroVelocity.getAsBoolean()) {
      pivotIO.setHomePosition(PivotPosition.stow.angle);
      pivotIO.setPosition(PivotPosition.stow.angle);
      homedSinceLastSetpoint = true;
      stopHoming();
    }

    Logger.recordOutput("Intake/Homing/waitForHoming/elapsed", waitForHoming.get());
    Logger.recordOutput("Intake/Homing/waitForHoming/running", waitForHoming.isRunning());

    Logger.recordOutput("Intake/Homing/homingDelay/elapsed", homingDelay.get());
    Logger.recordOutput("Intake/Homing/homingDelay/running", homingDelay.isRunning());

    Logger.recordOutput("Intake/SetpointPosition", lastSetpoint.angle);
  }

  @AutoLogOutput(key = "Intake/Homing/isHoming")
  public boolean isHoming() {
    return waitForHoming.isRunning() || homingDelay.isRunning();
  }

  private void startWaitForHoming() {
    homingDelay.stop();
    homingDelay.reset();
    waitForHoming.restart();
  }

  private void startHoming() {
    waitForHoming.stop();
    waitForHoming.reset();
    homingDelay.restart();
    pivotIO.setPosition(PivotConstants.homingAngle);
  }

  private void stopHoming() {
    waitForHoming.stop();
    waitForHoming.reset();
    homingDelay.stop();
    homingDelay.reset();
  }

  private void stopRollers() {
    rollerIO.stop();
    passthroughIO.stop();
  }

  @AutoLogOutput(key = "Intake/ShouldSpinIntakeRollers")
  public boolean shouldIntakeSpin() {
    return pivotInputs.position.gt(Rotations.of(-0.18));
  }

  private void intakeRollers() {
    if (shouldIntakeSpin()) {
      rollerIO.setTorque(
          IntakeConstants.intakeTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);
    }

    passthroughIO.setTorque(
        IntakeConstants.passthroughTorqueCurrent, IntakeConstants.passthroughTorqueDutyCycle);
  }

  private void reverseRollers() {
    if (shouldIntakeSpin()) {
      rollerIO.setTorque(
          IntakeConstants.intakeReverseTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);
    }

    passthroughIO.setTorque(
        IntakeConstants.passthroughReverseTorqueCurrent,
        IntakeConstants.passthroughTorqueDutyCycle);
  }

  public Command intake() {
    return runEnd(this::intakeRollers, this::stopRollers);
  }

  public Command reverse() {
    return runEnd(this::reverseRollers, this::stopRollers);
  }

  public Command setPosition(PivotPosition position) {
    return run(() -> {
          stopHoming();
          homedSinceLastSetpoint = false;
          pivotIO.setPosition(position.angle);
          lastSetpoint = position;
        })
        .until(() -> position.withinTolerance(pivotInputs.position));
  }

  public Command toggleDeploy() {
    return Commands.either(
        setPosition(PivotPosition.deploy),
        setPosition(PivotPosition.stow),
        () -> {
          return (lastSetpoint != PivotPosition.deploy);
        });
  }
}
