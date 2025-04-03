package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants.ClimberPosition;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberInputsAutoLogged climberInputs = new ClimberInputsAutoLogged();

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
  }

  private void stop() {
    climberIO.setVoltage(Volts.zero());
  }

  public Command setPosition(ClimberPosition position) {
    return run(() -> {
          climberIO.setPosition(position.angle);
        })
        .until(() -> position.withinTolerance(climberInputs.position));
  }

  public Command manualControl(DoubleSupplier outputSupplier) {
    return runEnd(
        () ->
            climberIO.setVoltage(
                ClimberConstants.absManualControlVoltage.times(outputSupplier.getAsDouble())),
        this::stop);
  }
}
