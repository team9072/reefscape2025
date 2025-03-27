package frc.robot.commands.autos;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.commands.scoring.ScoringPosition;
import frc.robot.commands.ReefAlignment;
import frc.robot.commands.autos.chooser.AutoBuilder;
import frc.robot.subsystems.drive.Drive.DrivePid;
import frc.robot.subsystems.intake.PivotConstants.PivotPosition;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Autos extends SubsystemBase {
  private static class AutoInputs implements LoggableInputs {
    private final AutoChooser autoChooser;
    private final AutoBuilder builder = new AutoBuilder("AutoBuilder");

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

  private static final String prepareAlignEvent = "PrepareAlign";

  private final AutoFactory autoFactory;
  private final AutoInputs inputs;
  private final Robot.Subsystems s;

  private final DrivePid driveController;

  public Autos(Robot.Subsystems s) {
    driveController = s.drive.getPid();

    this.autoFactory =
        new AutoFactory(
            s.drive::getPose,
            (pose) -> {
              s.drive.setPose(pose);
              s.questNav.resetPose(pose);
            },
            this::drivePath,
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

  private void drivePath(SwerveSample sample) {
    ChassisSpeeds targetSpeeds = sample.getChassisSpeeds();

    Pose2d samplePose = new Pose2d(sample.x, sample.y, Rotation2d.fromRadians(sample.heading));
    targetSpeeds = targetSpeeds.plus(driveController.getPoseCorrection(samplePose));

    s.drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, s.drive.getRotation()));
  }

  private Command drivePose(Pose2d targetPose) {
    return Commands.run(
            () -> {
              ChassisSpeeds targetSpeeds =
                  ReefAlignment.getReefAlignSpeeds(targetPose, driveController);

              s.drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds, s.drive.getRotation()));
            },
            s.drive)
        .until(() -> driveController.posePidAtSetpoint(targetPose));
  }

  private Command completeAlign(AutoTrajectory trajectory) {
    return drivePose(trajectory.getFinalPose().get());
  }

  private void scorePreload(AutoTrajectory trajectory, Command afterScore) {
    trajectory.active().onTrue(s.scoring.stowHold());

    trajectory.atTime(prepareAlignEvent).onTrue(s.scoring.prepareElevator(ScoringPosition.branchL4));

    trajectory
        .done()
        .onTrue(
            Commands.sequence(
                completeAlign(trajectory),
                s.scoring.scoringActionAuto(ScoringPosition.branchL4),
                Commands.parallel(s.scoring.elevatorDown(), afterScore)));
  }

  private void intakeAndScore(
      AutoTrajectory intakeTrajectory,
      AutoTrajectory prepareTrajectory,
      AutoTrajectory scoreTrajectory,
      Command afterScore) {
    intakeTrajectory
        .active()
        .onTrue(
            s.intake.setPosition(PivotPosition.deploy).withTimeout(0).andThen(s.intake.intake()));

    intakeTrajectory
        .active()
        .and(s.intake.coralInPassthrough.or(s.intake.coralStaged))
        .onTrue(prepareTrajectory.cmd());
    intakeTrajectory.chain(prepareTrajectory);

    prepareTrajectory
        .done()
        .onTrue(
            Commands.sequence(
                s.scoring.grabCoral(),
                s.scoring.prepareElevator(ScoringPosition.branchL4),
                scoreTrajectory.cmd()));

    scoreTrajectory
        .recentlyDone()
        .onTrue(
            Commands.sequence(
                completeAlign(scoreTrajectory),
                s.scoring.scoringActionAuto(ScoringPosition.branchL4),
                Commands.parallel(s.scoring.elevatorDown(), afterScore)));
  }

  private AutoRoutine preload1p(String name, String preloadTrajName) {
    AutoRoutine routine = autoFactory.newRoutine(name);
    AutoTrajectory scorePreloadTraj = routine.trajectory(preloadTrajName);

    routine
        .active()
        .onTrue(Commands.sequence(scorePreloadTraj.resetOdometry(), scorePreloadTraj.spawnCmd()));

    scorePreload(scorePreloadTraj, Commands.none());

    return routine;
  }

  private AutoRoutine preload2p(
      String name, String preloadTrajName, String intakeAndScoreSecondTrajName) {
    AutoRoutine routine = autoFactory.newRoutine(name);
    AutoTrajectory scorePreloadTraj = routine.trajectory(preloadTrajName);
    AutoTrajectory intakeSecondTraj = routine.trajectory(intakeAndScoreSecondTrajName, 0);
    AutoTrajectory prepareSecondTraj = routine.trajectory(intakeAndScoreSecondTrajName, 1);
    AutoTrajectory scoreSecondTraj = routine.trajectory(intakeAndScoreSecondTrajName, 2);

    routine
        .active()
        .onTrue(Commands.sequence(scorePreloadTraj.resetOdometry(), scorePreloadTraj.spawnCmd()));

    scorePreload(scorePreloadTraj, intakeSecondTraj.spawnCmd());
    intakeAndScore(intakeSecondTraj, prepareSecondTraj, scoreSecondTraj, Commands.none());

    return routine;
  }

  private AutoRoutine preload3p(
      String name,
      String preloadTrajName,
      String intakeAndScoreSecondTrajName,
      String intakeAndScoreThirdTrajName) {
    AutoRoutine routine = autoFactory.newRoutine(name);
    AutoTrajectory scorePreloadTraj = routine.trajectory(preloadTrajName);

    AutoTrajectory intakeSecondTraj = routine.trajectory(intakeAndScoreSecondTrajName, 0);
    AutoTrajectory scoreSecondTraj = routine.trajectory(intakeAndScoreSecondTrajName, 1);

    AutoTrajectory intakeThirdTraj = routine.trajectory(intakeAndScoreThirdTrajName, 0);
    AutoTrajectory scoreThirdTraj = routine.trajectory(intakeAndScoreThirdTrajName, 1);

    routine
        .active()
        .onTrue(Commands.sequence(scorePreloadTraj.resetOdometry(), scorePreloadTraj.spawnCmd()));

    scorePreload(scorePreloadTraj, intakeSecondTraj.spawnCmd());
    // TODO: FIXME intakeAndScore(intakeSecondTraj, scoreSecondTraj, intakeThirdTraj.spawnCmd());
    // TODO: FIXME intakeAndScore(intakeThirdTraj, scoreThirdTraj, Commands.none());

    return routine;
  }

  private AutoChooser buildAutoChooser() {
    AutoChooser autoChooser = new AutoChooser();

    /** C2S (Alliance color start) */
    autoChooser.addRoutine("C2S [Aliance color] 1p", () -> preload1p("C2S 1p", "C2S to R2P1"));
    autoChooser.addRoutine(
        "C2S [Alliance color] 2p", () -> preload2p("C2S 2p", "C2S to R2P1", "R2P1 to S1 to R3P2"));
    /*TODO: FIXME autoChooser.addRoutine(
    "C2S [Alliance color] 3p",
    () ->
        preload3p(
            "C2S [Alliance color] 3p",
            "C2S to R2P1",
            "R2P1 to S1 to R3P2",
            "R3P2 to S2 to R4P1"));*/

    /** C5S (Opposite color start) */
    autoChooser.addRoutine("C5S [Opposite color] 1p", () -> preload1p("C5S 1p", "C5S to R6P2"));
    autoChooser.addRoutine(
        "C5S [Opposite color] 2p", () -> preload2p("C5S 2p", "C5S to R6P2", "R6P2 to S3 to R5P1"));
    /*TODO: FIXME autoChooser.addRoutine(
    "C5S [Opposite color] 3p",
    () ->
        preload3p(
            "C5S [Opposite] 3p", "C5S to R6P2", "R6P2 to S3 to R5P1", "R5P1 to S2 to R4P1"));*/

    SmartDashboard.putData("Selected Auto", autoChooser);
    return autoChooser;
  }
}
