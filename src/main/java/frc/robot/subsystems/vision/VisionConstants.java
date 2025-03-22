// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.HashMap;
import java.util.Optional;

public class VisionConstants {
  public enum CameraData {
    LeftCamera(
        "LeftCamera",
        new Transform3d(
            new Translation3d(Inches.of(4.5), Inches.of(11.75), Inches.of(18.5)),
            new Rotation3d(Degrees.zero(), Degrees.of(20), Degrees.of(-30)))),

    RightCamera(
        "RightCamera",
        new Transform3d(
            new Translation3d(Inches.of(4.5), Inches.of(-11.75), Inches.of(18.5)),
            new Rotation3d(Degrees.zero(), Degrees.of(20), Degrees.of(30))));

    private static final HashMap<String, CameraData> _map = new HashMap<>();

    public final String cameraName;
    public final Transform3d robotToCamera;
    public final double stdDevFactor;

    public static Optional<CameraData> getNamed(String name) {
      return Optional.ofNullable(_map.get(name));
    }

    CameraData(String cameraName, Transform3d robotToCamera, double stdDevFactor) {
      this.cameraName = cameraName;
      this.robotToCamera = robotToCamera;
      this.stdDevFactor = stdDevFactor;
    }

    CameraData(String cameraName, Transform3d robotToCamera) {
      this(cameraName, robotToCamera, 1);
    }
  }

  public enum ReefTags {
    blue(17, 22),
    red(6, 11);

    private final int first;
    private final int last;

    ReefTags(int first, int last) {
      this.first = first;
      this.last = last;
    }

    boolean includesTag(int tagId) {
      return tagId >= first && tagId <= last;
    }

    static boolean allianceIncludesTags(int tagId) {
      if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
        return ReefTags.blue.includesTag(tagId);
      } else {
        return ReefTags.red.includesTag(tagId);
      }
    }
  }

  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.1; // Meters
  public static double angularStdDevBaseline = 0.085; // Radians
}
