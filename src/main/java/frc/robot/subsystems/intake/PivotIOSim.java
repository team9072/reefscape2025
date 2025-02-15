package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;

public class PivotIOSim implements PivotIO {
  private final SingleJointedArmSim armSim;

  private final MutVoltage appliedVoltage = Volts.mutable(0);

  private ProfiledPIDController angleController =
      new ProfiledPIDController(
          PivotConstants.kP,
          0,
          0,
          new TrapezoidProfile.Constraints(
              PivotConstants.rampAcceleration, PivotConstants.rampAcceleration));
  private boolean usePid = false;

  public PivotIOSim(PivotConstants constants) {
    DCMotor motor = DCMotor.getKrakenX60(1);

    Angle minAngle = PivotPosition.stow.angle;
    Angle maxAngle = PivotPosition.stow.angle;

    armSim =
        new SingleJointedArmSim(
            LinearSystemId.createSingleJointedArmSystem(
                motor,
                PivotConstants.armMoi.in(KilogramSquareMeters),
                PivotConstants.motorReduction),
            motor,
            PivotConstants.motorReduction,
            PivotConstants.armLength.in(Meters),
            minAngle.in(Radians),
            maxAngle.in(Radians),
            false,
            PivotPosition.stow.angle.in(Radians));
  }

  private void setVoltageClamped(double voltage) {
    appliedVoltage.mut_replace(MathUtil.clamp(voltage, -12.0, 12.0), Volts);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (usePid) {
      setVoltageClamped(angleController.calculate(armSim.getAngleRads()));
    }

    armSim.setInputVoltage(appliedVoltage.in(Volts));
    armSim.update(0.02);

    inputs.position = Radians.of(armSim.getAngleRads());
    inputs.velocity = RadiansPerSecond.of(armSim.getVelocityRadPerSec());

    inputs.appliedVoltage = appliedVoltage.copy();
    inputs.current = Amps.of(armSim.getCurrentDrawAmps());
  }
}
