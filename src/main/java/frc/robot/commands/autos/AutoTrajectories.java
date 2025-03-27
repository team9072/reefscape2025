package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.autos.AutoPositions.AutoPosition;
import frc.robot.commands.autos.AutoPositions.CoralSpike;
import frc.robot.commands.autos.AutoPositions.ReefPole;
import frc.robot.commands.autos.AutoPositions.StartingPosition;
import java.util.Optional;

public class AutoTrajectories {
  public interface AutoTrajectory {
    String name();

    String displayName();

    AutoPosition initialPosition();

    AutoPosition finalPosition();

    Trajectory<SwerveSample> trajectory();

    default Optional<CoralSpike> spikeTaken() {
      return Optional.empty();
    }
  }

  private static Trajectory<SwerveSample> loadTrajectory(String trajName) {
    var optTrajectory = Choreo.<SwerveSample>loadTrajectory(trajName);

    if (optTrajectory.isPresent()) {
      return optTrajectory.get();
    } else {
      throw new Error("Trajectory `" + trajName + "` not found");
    }
  }

  public enum PreloadTrajectory implements AutoTrajectory {
    C2S_I(StartingPosition.C2S, ReefPole.I),
    C5S_F(StartingPosition.C5S, ReefPole.F);

    public final StartingPosition startingPosition;
    public final ReefPole endingPole;
    public final Trajectory<SwerveSample> trajectory;

    PreloadTrajectory(StartingPosition startingPosition, ReefPole endingPole) {
      this.startingPosition = startingPosition;
      this.endingPole = endingPole;

      this.trajectory = loadTrajectory(startingPosition.name() + " to " + endingPole.name());
    }

    @Override
    public String displayName() {
      return startingPosition.displayName() + " to " + endingPole.displayName();
    }

    @Override
    public AutoPosition initialPosition() {
      return startingPosition;
    }

    @Override
    public AutoPosition finalPosition() {
      return endingPole;
    }

    @Override
    public Trajectory<SwerveSample> trajectory() {
      return trajectory;
    }
  }

  public enum IntakeTrajectory implements AutoTrajectory {
    I_S1_L(ReefPole.I, CoralSpike.S1, ReefPole.L),
    L_S2_B(ReefPole.L, CoralSpike.S2, ReefPole.B),
    C_S2_A(ReefPole.C, CoralSpike.S2, ReefPole.A),
    F_S3_C(ReefPole.F, CoralSpike.S3, ReefPole.C);

    public final ReefPole startingPole;
    public final CoralSpike coralSpike;
    public final ReefPole endingPole;
    public final Trajectory<SwerveSample> trajectory;

    IntakeTrajectory(ReefPole startingPole, CoralSpike coralSpike, ReefPole endingPole) {
      this.startingPole = startingPole;
      this.coralSpike = coralSpike;
      this.endingPole = endingPole;

      this.trajectory =
          loadTrajectory(
              startingPole.name() + " to " + coralSpike.name() + " to " + endingPole.name());
    }

    @Override
    public String displayName() {
      return startingPole.displayName()
          + " to "
          + coralSpike.displayName()
          + " to "
          + endingPole.displayName();
    }

    @Override
    public AutoPosition initialPosition() {
      return startingPole;
    }

    @Override
    public AutoPosition finalPosition() {
      return endingPole;
    }

    @Override
    public Trajectory<SwerveSample> trajectory() {
      return trajectory;
    }

    @Override
    public Optional<CoralSpike> spikeTaken() {
      return Optional.of(coralSpike);
    }
  }
}
