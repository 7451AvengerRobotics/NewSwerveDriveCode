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

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Field {
    public static double fieldWidth = Units.feetToMeters(26.0) + Units.inchesToMeters(5.0);
    public static double fieldLength = Units.feetToMeters(57.0) + Units.inchesToMeters(6.875);

    public static Pose2d apply(Pose2d reef) {
      return new Pose2d(
          fieldLength - reef.getX(),
          fieldWidth - reef.getY(),
          reef.getRotation().rotateBy(Rotation2d.kPi));
    }
  }

  public static class Reef {
    public static final Pose2d reef0 =
        new Pose2d(
            Units.inchesToMeters(144.0945),
            Units.inchesToMeters(158.6614),
            Rotation2d.fromDegrees(0));
    public static final Pose2d reef1 =
        new Pose2d(
            Units.inchesToMeters(160.373),
            Units.inchesToMeters(186.857),
            Rotation2d.fromDegrees(300));
    public static final Pose2d reef2 =
        new Pose2d(
            Units.inchesToMeters(193.116),
            Units.inchesToMeters(186.858),
            Rotation2d.fromDegrees(240));
    public static final Pose2d reef3 =
        new Pose2d(
            Units.inchesToMeters(209.489),
            Units.inchesToMeters(158.502),
            Rotation2d.fromDegrees(180));
    public static final Pose2d reef4 =
        new Pose2d(
            Units.inchesToMeters(193.118),
            Units.inchesToMeters(130.145),
            Rotation2d.fromDegrees(120));

    public static final Pose2d reef5 =
        new Pose2d(
            Units.inchesToMeters(160.375),
            Units.inchesToMeters(130.144),
            Rotation2d.fromDegrees(60));

    public static final Pose2d[] blueReefs = {reef0, reef1, reef2, reef3, reef4, reef5};

    public static final Pose2d[] redReefs = {
      Field.apply(reef0),
      Field.apply(reef1),
      Field.apply(reef2),
      Field.apply(reef3),
      Field.apply(reef4),
      Field.apply(reef5)
    };
  }
}
