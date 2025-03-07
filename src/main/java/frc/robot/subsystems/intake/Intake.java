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

  private final RollerIO stagingIO;
  private final RollerIOInputsAutoLogged stagingInputs = new RollerIOInputsAutoLogged();

  private final BeamBreakIO beamBreakIO;

  private final BeamBreakIOInputsAutoLogged beamBreakInputs = new BeamBreakIOInputsAutoLogged();

  private PivotPosition lastSetpoint = PivotPosition.stow;
  private Trigger isAtZeroVelocity =
      new Trigger(() -> pivotInputs.velocity.isEquivalent(RotationsPerSecond.zero())).debounce(0.2);

  private boolean homedSinceLastSetpoint = false;
  private Timer waitForHoming = new Timer();
  private Timer homingDelay = new Timer();

  public Trigger coralDetected = new Trigger(() -> beamBreakInputs.objectDetected);

  public Intake(
      PivotIO pivotIO,
      RollerIO rollerIO,
      RollerIO passthroughIO,
      RollerIO stagingIO,
      BeamBreakIO beamBreakIO) {
    this.pivotIO = pivotIO;
    this.rollerIO = rollerIO;
    this.passthroughIO = passthroughIO;
    this.stagingIO = stagingIO;
    this.beamBreakIO = beamBreakIO;

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

    beamBreakIO.updateInputs(beamBreakInputs);
    Logger.processInputs("Intake/Beam Break", beamBreakInputs);

    if (lastSetpoint.shouldFloat && lastSetpoint.withinTolerance(pivotInputs.position)) {
      pivotIO.setFloating();
    }

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
    stagingIO.stop();
  }

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

    stagingIO.setTorque(
        IntakeConstants.stagingTorqueCurrent, IntakeConstants.stagingTorqueDutyCycle);
  }

  private void reverseRollers() {
    if (shouldIntakeSpin()) {
      rollerIO.setTorque(
          IntakeConstants.intakeReverseTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle);
    }

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

  public Command idleStagingRoller() {
    return this.runEnd(
        () ->
            stagingIO.setTorque(
                IntakeConstants.stagingIdleTorqueCurrent, IntakeConstants.stagingIdleDutyCycle),
        () -> stagingIO.stop());
  }

  public Command reverse() {
    return runEnd(this::reverseRollers, this::stopRollers);
  }

  public Command reverseAlgae() {
    return runEnd(
        () ->
            rollerIO.setTorque(
                IntakeConstants.intakeTorqueCurrent, IntakeConstants.intakeTorqueDutyCycle),
        this::stopRollers);
  }

  public Command setPosition(PivotPosition position) {
    return Commands.sequence(
        runOnce(
            () -> {
              stopHoming();
              homedSinceLastSetpoint = false;
              pivotIO.setPosition(position.angle);
              lastSetpoint = position;
            }),
        Commands.waitUntil(() -> position.withinTolerance(pivotInputs.position)));
  }

  public Command toggleDeploy(Command onDeploy) {
    return Commands.either(
        setPosition(PivotPosition.stow),
        onDeploy.andThen(setPosition(PivotPosition.deploy)),
        () -> {
          return (lastSetpoint != PivotPosition.stow);
        });
  }
}
