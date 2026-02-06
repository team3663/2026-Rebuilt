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
    public static final Pose2d BLUE_LEFT_AUTO_LINE = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) + 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(2.0), FIELD.getTagPose(23).get().getRotation().toRotation2d());
    public static final Pose2d BLUE_RIGHT_AUTO_LINE = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters(((47.0 / 2) + 13.0)),
            Units.feetToMeters(2.0), FIELD.getTagPose(28).get().getRotation().toRotation2d());
    public static final Pose2d BLUE_LEFT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(2.0), FIELD.getTagPose(23).get().getRotation().toRotation2d().plus(new Rotation2d(Units.degreesToRadians(180.0))));
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(2.0), FIELD.getTagPose(28).get().getRotation().toRotation2d().plus(new Rotation2d(Units.degreesToRadians(180.0))));
    public static final Pose2d BLUE_DEPOT_AUTO_LINE = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) + 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(4.0), FIELD.getTagPose(23).get().toPose2d().getRotation());

    // Red Alliance auto starting positions
    public static final Pose2d RED_LEFT_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(2.0), FIELD.getTagPose(7).get().getRotation().toRotation2d());
    public static final Pose2d RED_RIGHT_AUTO_LINE = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(2.0), FIELD.getTagPose(12).get().getRotation().toRotation2d());
    public static final Pose2d RED_LEFT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(2.0), FIELD.getTagPose(7).get().getRotation().toRotation2d().plus(new Rotation2d(Units.degreesToRadians(180.0))));
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(2.0), FIELD.getTagPose(12).get().getRotation().toRotation2d().plus(new Rotation2d(Units.degreesToRadians(180.0))));
    public static final Pose2d RED_DEPOT_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2.0) + 13.0),
            Units.feetToMeters(4.0), FIELD.getTagPose(7).get().toPose2d().getRotation());

    // TODO figure out distance away from outpost - currently set to 2 in
    // Outpost and Depot Positions
    public static final Pose2d BLUE_OUTPOST_CENTERED = FIELD.getTagPose(29).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(13.0 + 12.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_OUTPOST_INTERMEDIATE = BLUE_OUTPOST_CENTERED.plus(new Transform2d(-Units.feetToMeters(6.0), -Units.feetToMeters(5.0), new Rotation2d(Units.degreesToRadians(45))));
    public static final Pose2d RED_OUTPOST_CENTERED = FIELD.getTagPose(13).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(13.0 + 12.0), 0.0, new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d RED_OUTPOST_INTERMEDIATE = RED_OUTPOST_CENTERED.plus(new Transform2d(-Units.feetToMeters(6.0), -Units.feetToMeters(5.0), new Rotation2d(Units.degreesToRadians(-45))));

    public static final Pose2d BLUE_RIGHT_DEPOT = new Pose2d(Units.inchesToMeters(27.0 / 2.0), (FIELD.getFieldWidth() / 2.0) + Units.inchesToMeters(75.93) - Units.inchesToMeters(42.0 / 2.0),
            new Rotation2d(Units.degreesToRadians(270)));
    public static final Pose2d BLUE_LEFT_DEPOT = new Pose2d(Units.inchesToMeters(27.0 / 2.0), ((FIELD.getFieldWidth() / 2.0) + Units.inchesToMeters(75.93) + Units.inchesToMeters(42.0 / 2.0)),
            new Rotation2d(Units.degreesToRadians(270)));
    public static final Pose2d RED_RIGHT_DEPOT = new Pose2d(FIELD.getFieldLength() - Units.inchesToMeters(27.0 / 2.0), (FIELD.getFieldWidth() / 2.0) - Units.inchesToMeters(75.93) + Units.inchesToMeters(42.0 / 2.0),
            new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d RED_LEFT_DEPOT = new Pose2d(FIELD.getFieldLength() - Units.inchesToMeters(27.0 / 2.0), (FIELD.getFieldWidth() / 2.0) - Units.inchesToMeters(75.93) - Units.inchesToMeters(42.0 / 2.0),
            new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d RED_AROUND_TOWER_TO_OUTPOST_INTERMEDIATE = FIELD.getTagPose(16).get().toPose2d()
            .plus(new Transform2d(Units.feetToMeters(12.5), Units.feetToMeters(1.0), new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_AROUND_TOWER_TO_OUTPOST_INTERMEDIATE = FIELD.getTagPose(32).get().toPose2d()
            .plus(new Transform2d(Units.feetToMeters(12.5), -Units.feetToMeters(1.0), new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d RED_AROUND_TOWER_TO_DEPOT_INTERMEDIATE = FIELD.getTagPose(16).get().toPose2d()
            .plus(new Transform2d(Units.feetToMeters(10.0), Units.feetToMeters(10.0), new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_AROUND_TOWER_TO_DEPOT_INTERMEDIATE = FIELD.getTagPose(32).get().toPose2d()
            .plus(new Transform2d(Units.feetToMeters(10.0), Units.feetToMeters(10.0), new Rotation2d(Units.degreesToRadians(180))));
    public static final Pose2d BLUE_STARTING_TO_DEPOT_INTERMEDIATE = BLUE_DEPOT_AUTO_LINE.plus(new Transform2d(Units.feetToMeters(5.0), 0.0, new Rotation2d(0.0)));
    public static final Pose2d RED_STARTING_TO_DEPOT_INTERMEDIATE = RED_DEPOT_AUTO_LINE.plus(new Transform2d(Units.feetToMeters(5.0), 0.0, new Rotation2d(0.0)));

    // Climb Positions
    public static final Pose2d BLUE_LEFT_RUNG_CLIMB = FIELD.getTagPose(31).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d BLUE_RIGHT_RUNG_CLIMB = FIELD.getTagPose(31).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), -Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d RED_LEFT_RUNG_CLIMB = FIELD.getTagPose(15).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d RED_RIGHT_RUNG_CLIMB = FIELD.getTagPose(15).get().toPose2d().plus(
            new Transform2d(Units.inchesToMeters(43.510 + 13.0 + 2.0), -Units.inchesToMeters(32.25 + (1.5 / 2.0)), new Rotation2d(Units.degreesToRadians(0))));
    public static final Pose2d BLUE_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE = FIELD.getTagPose(28).get().toPose2d().plus(
            new Transform2d(Units.feetToMeters(7.0), 0.0, BLUE_RIGHT_AUTO_LINE.getRotation()));
    public static final Pose2d RED_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE = FIELD.getTagPose(12).get().toPose2d().plus(
            new Transform2d(Units.feetToMeters(7.0), 0.0, RED_RIGHT_UNDER_TRENCH_AUTO_LINE.getRotation()));
    public static final Pose2d BLUE_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE = FIELD.getTagPose(23).get().toPose2d().plus(
            new Transform2d(Units.feetToMeters(7.0), 0.0, BLUE_LEFT_AUTO_LINE.getRotation()));
    public static final Pose2d RED_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE = FIELD.getTagPose(7).get().toPose2d().plus(
            new Transform2d(Units.feetToMeters(7.0), 0.0, RED_LEFT_UNDER_TRENCH_AUTO_LINE.getRotation()));

    // Neutral Zone Positions
    public static final Pose2d BLUE_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(17).get().getY(), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_RIGHT_CENTER_LINE_TO_TRENCH = BLUE_RIGHT_CENTER_LINE.plus(new Transform2d(0.0, -Units.feetToMeters(2.5), new Rotation2d(Units.degreesToRadians(90))));
    public static final Pose2d BLUE_MIDDLE_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getFieldWidth() / 2.0, new Rotation2d(Units.degreesToRadians(90)));
    public static final Pose2d BLUE_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(22).get().getY(), new Rotation2d(Units.degreesToRadians(0)));
    public static final Pose2d BLUE_LEFT_CENTER_LINE_TO_TRENCH = BLUE_LEFT_CENTER_LINE.plus(new Transform2d(0.0, Units.feetToMeters(2.5), new Rotation2d(Units.degreesToRadians(90))));
    public static final Pose2d RED_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(1).get().getY(), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_MIDDLE_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getFieldWidth() / 2.0, new Rotation2d(Units.degreesToRadians(270)));
    public static final Pose2d RED_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0), FIELD.getTagPose(6).get().getY(), new Rotation2d(Units.degreesToRadians(180)));
    public static final Pose2d RED_RIGHT_CENTER_LINE_TO_TRENCH = RED_RIGHT_CENTER_LINE.plus(new Transform2d(0.0, -Units.feetToMeters(2.5), new Rotation2d(Units.degreesToRadians(90))));
    public static final Pose2d RED_LEFT_CENTER_LINE_TO_TRENCH = RED_LEFT_CENTER_LINE.plus(new Transform2d(0.0, Units.feetToMeters(2.5), new Rotation2d(Units.degreesToRadians(90))));

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
