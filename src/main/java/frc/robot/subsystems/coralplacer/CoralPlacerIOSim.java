package frc.robot.subsystems.coralplacer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;

public class CoralPlacerIOSim extends CoralPlacerIOTalonFX {
  private final SingleJointedArmSim sim;
  private final TalonFXSimState simState;

  private final double motorReduction;

  public CoralPlacerIOSim() {
    super();

    motorReduction = CoralPlacerConstants.motorReduction;
    simState = motor.getSimState();

    DCMotor motor = DCMotor.getKrakenX60(1);

    Angle minAngle = PivotPosition.stow.angle;
    Angle maxAngle = PivotPosition.stow.angle;

    sim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                motor, CoralPlacerConstants.armMoi.in(KilogramSquareMeters), motorReduction),
            motor,
            motorReduction,
            CoralPlacerConstants.armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            false,
            PivotPosition.stow.angle.in(Radians));
  }

  @Override
  public void updateInputs(CoralPlacerInputs inputs) {
    simState.setSupplyVoltage(RobotController.getBatteryVoltage());
    sim.setInputVoltage(simState.getMotorVoltage());
    sim.update(0.02);

    simState.setRawRotorPosition(motorReduction * sim.getAngleRads());
    simState.setRotorVelocity(
        motorReduction * Units.radiansToRotations(sim.getVelocityRadPerSec()));

    super.updateInputs(inputs);
  }
}
