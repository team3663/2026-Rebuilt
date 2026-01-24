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
    // TODO get the 2026 one:
    public static final boolean IS_ANDYMARK = false;
    public static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public enum Mode {
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

    public static class Shooter {
        // TODO: get actual values
        // The Pose2d's of the six locations we will want to pass to
        public static final Pose2d RED_HUB = new Pose2d(4.4, 4.0, Rotation2d.kZero);
        public static final Pose2d BLUE_HUB = new Pose2d(11.8, 4.0, Rotation2d.kZero);

        public static final Pose2d UPPER_PASS_RED = new Pose2d(0.0, 8.0, Rotation2d.kZero);
        public static final Pose2d LOWER_PASS_RED = new Pose2d(0.0, 0.0, Rotation2d.kZero);
        public static final Pose2d UPPER_PASS_BLUE = new Pose2d(16.5, 8.0, Rotation2d.kZero);
        public static final Pose2d LOWER_PASS_BLUE = new Pose2d(16.5, 0.0, Rotation2d.kZero);

        // The heights we will attempt to pass to when passing to the hub and our alliance zone, respectively
        public static final double HUB_HEIGHT = Units.inchesToMeters(45.0);
        public static final double PASS_HEIGHT = Units.feetToMeters(1.0);

        // The Radians from the turrets 0 position to the angle of turrets location around the center of the robot
        public static final double TURRET_OFFSET_ANGLE = Units.degreesToRadians(45.0);
        // The meters the center of the turret is from the center of the robot
        public static final double TURRET_OFFSET_DISTANCE = Units.inchesToMeters(Math.sqrt(365) - 4.0);
        public static final double LAUNCH_HEIGHT = Units.inchesToMeters(21.0);
    }
}
