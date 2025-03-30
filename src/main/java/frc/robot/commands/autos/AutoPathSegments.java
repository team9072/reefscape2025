package frc.robot.commands.autos;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import frc.robot.commands.autos.AutoPositions.AutoPosition;
import frc.robot.commands.autos.AutoPositions.CoralSpike;
import frc.robot.commands.autos.AutoPositions.ReefPole;
import frc.robot.commands.autos.AutoPositions.StartingPosition;
import java.util.Optional;

public class AutoPathSegments {
  public interface AutoPathSegment {
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

  public enum PreloadTrajectory implements AutoPathSegment {
    C2S_I(StartingPosition.C2S, ReefPole.I),
    C5S_F(StartingPosition.C5S, ReefPole.F),
    C1S_A(StartingPosition.C1S, ReefPole.A),
    C1S_B(StartingPosition.C1S, ReefPole.B),
    C6S_A(StartingPosition.C6S, ReefPole.A),
    C6S_B(StartingPosition.C6S, ReefPole.B);

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

  public enum IntakeTrajectory implements AutoPathSegment {
    C_S2_A(ReefPole.C, CoralSpike.S2, ReefPole.A),
    F_S3_C(ReefPole.F, CoralSpike.S3, ReefPole.C),
    I_S1_L(ReefPole.I, CoralSpike.S1, ReefPole.L),
    L_S2_B(ReefPole.L, CoralSpike.S2, ReefPole.B),
    A_S2_B(ReefPole.A, CoralSpike.S2, ReefPole.B),
    A_S1_B(ReefPole.A, CoralSpike.S1, ReefPole.B),
    B_S2_A(ReefPole.B, CoralSpike.S2, ReefPole.A);

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
