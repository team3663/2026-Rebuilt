// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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

    public static final Boolean IS_ANDYMARK = false;
    // TODO get 2026 field when available
    public static AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(IS_ANDYMARK ? AprilTagFields.k2025ReefscapeAndyMark : AprilTagFields.k2025ReefscapeWelded);

    public static final Pose2d RED_AUTO_STARTING_6FT = new Pose2d(Units.inchesToMeters(156.61),
            Units.inchesToMeters(72.0 + 14.0), Rotation2d.fromDegrees(0));
    public static final Pose2d RED_AUTO_STARTING_CENTER = new Pose2d(Units.inchesToMeters(156.61),
            Units.inchesToMeters(FIELD.getFieldWidth() / 2.0), Rotation2d.fromDegrees(0));



    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }
}
