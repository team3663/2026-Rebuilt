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
import edu.wpi.first.math.geometry.Transform2d;
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
    public static AprilTagFieldLayout FIELD =
            AprilTagFieldLayout.loadField(IS_ANDYMARK ? AprilTagFields.k2026RebuiltAndymark : AprilTagFields.k2026RebuiltWelded);

    // Blue alliance auto starting positions
    public static final Pose2d BLUE_LEFT_AUTO_LINE = FIELD.getTagPose(23).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) + 13.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_RIGHT_AUTO_LINE = FIELD.getTagPose(28).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) + 13.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_LEFT_UNDER_TRENCH_AUTO_LINE = FIELD.getTagPose(23).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) - 13.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE = FIELD.getTagPose(28).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) - 13.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    // TODO do I want a center starting position?

    // Red Alliance auto starting positions
    public static final Pose2d RED_LEFT_AUTO_LINE = FIELD.getTagPose(7).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) + 13.0), 0.0, new Rotation2d(0)));
    public static final Pose2d RED_RIGHT_AUTO_LINE = FIELD.getTagPose(12).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) + 13.0), 0.0, new Rotation2d(0)));
    public static final Pose2d RED_LEFT_UNDER_TRENCH_AUTO_LINE = FIELD.getTagPose(7).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) - 13.0), 0.0, new Rotation2d(0)));
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_AUTO_LINE = FIELD.getTagPose(12).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters((47.0 / 2) - 13.0), 0.0, new Rotation2d(0)));

    // TODO figure out distance away from outpost - currently set to 2 in
    // Outpost and Depot Positions
    public static final Pose2d BLUE_OUTPOST_CENTERED = FIELD.getTagPose(29).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(13.0 + 2.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d RED_OUTPOST_CENTERED = FIELD.getTagPose(13).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(13.0 + 2.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_RIGHT_DEPOT = new Pose2d((FIELD.getFieldWidth() / 2.0) + Units.inchesToMeters(75.93) + Units.inchesToMeters(42.0 / 2.0),
            Units.inchesToMeters(27.0 / 2.0), new Rotation2d(270));
    public static final Pose2d BLUE_LEFT_DEPOT = new Pose2d((FIELD.getFieldWidth() / 2.0) + Units.inchesToMeters(75.93) - Units.inchesToMeters(42.0 / 2.0),
            Units.inchesToMeters(27.0 / 2.0), new Rotation2d(270));
    public static final Pose2d RED_RIGHT_DEPOT = new Pose2d((FIELD.getFieldWidth() / 2.0) - Units.inchesToMeters(75.93) - Units.inchesToMeters(42.0 / 2.0),
            Units.inchesToMeters(27.0 / 2.0), new Rotation2d(90));
    public static final Pose2d RED_LEFT_DEPOT = new Pose2d((FIELD.getFieldWidth() / 2.0) - Units.inchesToMeters(75.93) + Units.inchesToMeters(42.0 / 2.0),
            Units.inchesToMeters(27.0 / 2.0), new Rotation2d(90));

    // Climb Positions
    public static final Pose2d BLUE_LEFT_RUNG_CLIMB = FIELD.getTagPose(31).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), -Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d BLUE_RIGHT_RUNG_CLIMB = FIELD.getTagPose(31).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d RED_LEFT_RUNG_CLIMB = FIELD.getTagPose(15).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d RED_RIGHT_RUNG_CLIMB = FIELD.getTagPose(15).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), -Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(180))));

    // Neutral Zone Positions
    public static final Pose2d BLUE_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(17).get().getY(), new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d BLUE_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(22).get().getY(), new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d RED_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldWidth() / 2.0), FIELD.getTagPose(1).get().getY(),new Rotation2d(Units.degreesToRadians(270)));
    public static final Pose2d RED_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(6).get().getY(), new Rotation2d(Units.degreesToRadians(270)));

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
