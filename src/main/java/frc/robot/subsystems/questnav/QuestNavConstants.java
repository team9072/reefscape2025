package frc.robot.subsystems.questnav;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public class QuestNavConstants {
  public static final Distance translationStdDevs = Inches.of(2.0);
  public static final Angle rotationStdDevs = Degrees.of(20.0);
  public static final Vector<N3> stdDevs =
      VecBuilder.fill(
          translationStdDevs.in(Meters),
          translationStdDevs.in(Meters),
          rotationStdDevs.in(Radians));

  public static final Time setPoseDelay = Milliseconds.of(40.0);

  public static final Pose2d robotToQuest = new Pose2d();
}
