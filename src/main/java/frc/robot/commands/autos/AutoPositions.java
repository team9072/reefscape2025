package frc.robot.commands.autos;

public class AutoPositions {
  public interface AutoPosition {
    public String name();

    public String displayName();
  }

  public enum ReefSide {
    Front("Front"),
    FrontRight("Front Right"),
    BackRight("Back Right"),
    Back("Back"),
    BackLeft("Back Left"),
    FrontLeft("Front Left");

    public final String displayName;

    ReefSide(String displayName) {
      this.displayName = displayName;
    }
  }

  public enum ReefPole implements AutoPosition {
    A(ReefSide.Front),
    B(ReefSide.Front),
    C(ReefSide.FrontRight),
    D(ReefSide.FrontRight),
    E(ReefSide.BackRight),
    F(ReefSide.BackRight),
    G(ReefSide.Back),
    H(ReefSide.Back),
    I(ReefSide.BackLeft),
    J(ReefSide.BackLeft),
    K(ReefSide.FrontLeft),
    L(ReefSide.FrontLeft);

    public final ReefSide side;
    public final String displayName;

    ReefPole(ReefSide side) {
      this.side = side;
      this.displayName = side.displayName + " [" + name() + "]";
    }

    @Override
    public String displayName() {
      return displayName;
    }
  }

  public enum StartingPosition implements AutoPosition {
    C1S("Alliance Left Cage"),
    C2S("Alliance Center Cage"),
    C3S("Alliance Right Cage"),
    C4S("Opposite Left Cage"),
    C5S("Opposite Center Cage"),
    C6S("Opposite Right Cage");

    public final String displayName;

    StartingPosition(String displayName) {
      this.displayName = displayName + " [" + name() + "]";
    }

    @Override
    public String displayName() {
      return displayName;
    }
  }

  public enum CoralSpike implements AutoPosition {
    S1("Left Spike"),
    S2("Center Spike"),
    S3("Right Spike");

    public final String displayName;

    CoralSpike(String displayName) {
      this.displayName = displayName + " [" + name() + "]";
    }

    @Override
    public String displayName() {
      return displayName;
    }
  }
}
