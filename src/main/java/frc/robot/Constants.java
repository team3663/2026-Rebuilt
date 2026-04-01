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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.Logger;


/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
    public static final boolean IS_ANDYMARK = false;
    public static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(
            IS_ANDYMARK ? AprilTagFields.k2026RebuiltAndymark : AprilTagFields.k2026RebuiltWelded
    );

    public static final boolean ENABLE_TEST_FEATURES = false;

    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    // Distance away from the wall that the robot sits for auto paths beginning at the trench; measured in feet
    private static final double TRENCH_STARTING_OFFSET = 2.0;
    // Distance away from the middle line towards the driver station when crossing into the neutral zone
    //     for the 2nd time in auto; measured in feet
    private static final double NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET = 6.0;
    // Y Distance from the April Tags that are used when in the neutral zone on the left and right sides; measured in feet
    private static final double NEUTRAL_ZONE_Y_OFFSET = Units.feetToMeters(8.0);
    private static final double CENTER_LINE_INTERMEDIATE_X_OFFSET = 3.0;
    private static final double NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET = 7.0;
    private static final double NEUTRAL_ZONE_INTERMEDIATE_Y_OFFSET = 1.75;
    // X Distance the robot should sit away from the outpost
    private static final double OUTPOST_OFFSET = Units.inchesToMeters(24.0);
    private static final double BUMP_Y_OFFSET_FROM_CENTER = Units.inchesToMeters(62.0);
    private static final double BUMP_X_OFFSET_FROM_TRENCH = Units.inchesToMeters(23.5 + 14.0 + 6.0);
    private static final double DEPOT_Y_OFFSET_FROM_CENTER = Units.inchesToMeters(30.0);
    private static final double DEPOT_X_OFFSET = Units.inchesToMeters(14.25 + 5.0);

    // Rotation2d Constants
    private static final Rotation2d BLUE_ROTATED_LEFT = Rotation2d.fromDegrees(90.0);
    private static final Rotation2d BLUE_ROTATED_DOWNFIELD = Rotation2d.fromDegrees(0.0);
    private static final Rotation2d BLUE_ROTATED_RIGHT = Rotation2d.fromDegrees(270.0);
    private static final Rotation2d BLUE_ROTATED_TOWARDS_DRIVERSTATION = Rotation2d.fromDegrees(180.0);
    private static final Rotation2d BLUE_ROTATED_RIGHT_45_DEGREES = Rotation2d.fromDegrees(-20.0);
    private static final Rotation2d BLUE_ROTATED_LEFT_45_DEGREES = Rotation2d.fromDegrees(20.0);

    private static final Rotation2d RED_ROTATED_LEFT = Rotation2d.fromDegrees(270.0);
    private static final Rotation2d RED_ROTATED_DOWNFIELD = Rotation2d.fromDegrees(180.0);
    private static final Rotation2d RED_ROTATED_RIGHT = Rotation2d.fromDegrees(90.0);
    private static final Rotation2d RED_ROTATED_TOWARDS_DRIVERSTATION = Rotation2d.fromDegrees(0.0);
    private static final Rotation2d RED_ROTATED_RIGHT_45_DEGREES = Rotation2d.fromDegrees(20.0);
    private static final Rotation2d RED_ROTATED_LEFT_45_DEGREES = Rotation2d.fromDegrees(-20.0);

    // KEY NOTES:
    //     - If the robot is turned 90 or 270 degrees, X and Y are switched
    //     - X is the length of the field and Y is the width
    //     - The right blue corner of the field is (0,0)

    // Auto Starting Positions
    // BLUE ALLIANCE
    public static final Pose2d BLUE_LEFT_UNDER_TRENCH_AUTO_LINE_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_LEFT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE_ROTATED_LEFT = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE_ROTATED_TOWARDS_DRIVERSTATION = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_IN_FRONT_OF_HUB_AUTO_LINE = new Pose2d(
            FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) + 13.0),
            FIELD.getFieldWidth() / 2.0, BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_IN_FRONT_OF_BUMP_AUTO_LINE = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters(36.5),
            FIELD.getFieldWidth() / 2.0 + DEPOT_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_DOWNFIELD);

    // RED ALLIANCE
    public static final Pose2d RED_LEFT_UNDER_TRENCH_AUTO_LINE_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_RIGHT);
    public static final Pose2d RED_LEFT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_AUTO_LINE_ROTATED_LEFT = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_LEFT);
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_AUTO_LINE_ROTATED_TOWARDS_DRIVERSTATION = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_AUTO_LINE = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_IN_FRONT_OF_HUB_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2.0) + 13.0),
            FIELD.getFieldWidth() / 2.0, RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_IN_FRONT_OF_BUMP_AUTO_LINE = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters(36.5),
            FIELD.getFieldWidth() / 2.0 - DEPOT_Y_OFFSET_FROM_CENTER, RED_ROTATED_DOWNFIELD);

    // Neutral Zone Positions
    // BLUE ALLIANCE
    public static final Pose2d BLUE_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(1.5),
            FIELD.getTagPose(17).get().getY() + NEUTRAL_ZONE_Y_OFFSET, BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(1.5),
            FIELD.getTagPose(22).get().getY() - NEUTRAL_ZONE_Y_OFFSET, BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_RIGHT_CENTER_ALLIANCE_SIDE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(17).get().getY() + NEUTRAL_ZONE_Y_OFFSET + Units.feetToMeters(2.0), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_RIGHT_CENTER_ALLIANCE_SIDE_Y_OFFSET = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET) - Units.feetToMeters(2.0),
            FIELD.getTagPose(17).get().getY() + Units.feetToMeters(4.0), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_CENTER_ALLIANCE_SIDE_FACING_UPFIELD = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(17).get().getY() + NEUTRAL_ZONE_Y_OFFSET, BLUE_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d BLUE_LEFT_ALLIANCE_SIDE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(22).get().getY() - NEUTRAL_ZONE_Y_OFFSET - Units.feetToMeters(2.0), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_HUB_SHOOTING = BLUE_IN_FRONT_OF_HUB_AUTO_LINE.plus(new Transform2d(-Units.feetToMeters(3.5), 0.0, BLUE_ROTATED_DOWNFIELD));
    public static final Pose2d BLUE_LEFT_UNDER_TRENCH_SHOOTING = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0) - Units.feetToMeters(0.5),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_UNDER_TRENCH_SHOOTING = new Pose2d(FIELD.getTagPose(28).get().getX() - Units.inchesToMeters((47.0 / 2) - 13.0) - Units.feetToMeters(0.5),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(28).get().getX() - BUMP_X_OFFSET_FROM_TRENCH - Units.feetToMeters(1.0),
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d BLUE_RIGHT_BUMP_SHOOTING_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(28).get().getX() - BUMP_X_OFFSET_FROM_TRENCH - Units.feetToMeters(3.0),
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d BLUE_LEFT_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(23).get().getX() - BUMP_X_OFFSET_FROM_TRENCH - Units.feetToMeters(3.0),
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);

    public static final Pose2d BLUE_RIGHT_CENTER_LINE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(1.5),
            FIELD.getTagPose(17).get().getY() + Units.inchesToMeters(4.0), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_LEFT_CENTER_LINE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(1.0),
            FIELD.getTagPose(22).get().getY(), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(CENTER_LINE_INTERMEDIATE_X_OFFSET - 6.0) - Units.feetToMeters(1.5),
            FIELD.getTagPose(17).get().getY(), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(17).get().getY(), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_RIGHT = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(17).get().getY() + Units.feetToMeters(2.0), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_DOWNFIELD = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(17).get().getY(), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET = new Pose2d((FIELD.getTagPose(17).get().getX() + Units.feetToMeters(5.0)),
            FIELD.getTagPose(17).get().getY(), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET_ROTATED_LEFT = new Pose2d((FIELD.getTagPose(17).get().getX() + Units.feetToMeters(5.0)),
            FIELD.getTagPose(17).get().getY(), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET = new Pose2d((FIELD.getTagPose(17).get().getX() + Units.feetToMeters(5.0)),
            FIELD.getTagPose(22).get().getY(), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_ROTATED_RIGHT = new Pose2d((FIELD.getTagPose(17).get().getX() + Units.feetToMeters(5.0)),
            FIELD.getTagPose(22).get().getY(), BLUE_ROTATED_RIGHT);
    public static final Pose2d BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_Y_OFFSET = new Pose2d((FIELD.getTagPose(17).get().getX() + Units.feetToMeters(5.5)),
            FIELD.getTagPose(22).get().getY() - Units.feetToMeters(0.5), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET + 2.0),
            FIELD.getTagPose(22).get().getY(), BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_LEFT = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(22).get().getY(), BLUE_ROTATED_LEFT);
    public static final Pose2d BLUE_LEFT_BUMP_NZ = new Pose2d(FIELD.getTagPose(22).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d BLUE_LEFT_BUMP_AS = new Pose2d(FIELD.getTagPose(23).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d BLUE_RIGHT_BUMP_NZ = new Pose2d(FIELD.getTagPose(17).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d BLUE_RIGHT_BUMP_AS = new Pose2d(FIELD.getTagPose(28).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d BLUE_RIGHT_BUMP_NZ_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(17).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d BLUE_RIGHT_BUMP_AS_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(28).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_RIGHT_45_DEGREES);

    // RED ALLIANCE
    public static final Pose2d RED_RIGHT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(1.5),
            FIELD.getTagPose(1).get().getY() - NEUTRAL_ZONE_Y_OFFSET, RED_ROTATED_LEFT);
    public static final Pose2d RED_LEFT_CENTER_LINE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(1.5),
            FIELD.getTagPose(6).get().getY() + NEUTRAL_ZONE_Y_OFFSET, RED_ROTATED_RIGHT);
    public static final Pose2d RED_RIGHT_CENTER_ALLIANCE_SIDE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(1).get().getY() - NEUTRAL_ZONE_Y_OFFSET - Units.feetToMeters(2.0), RED_ROTATED_LEFT);
    public static final Pose2d RED_RIGHT_CENTER_ALLIANCE_SIDE_Y_OFFSET = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET) + Units.feetToMeters(2.0),
            FIELD.getTagPose(1).get().getY() - Units.feetToMeters(4.0), RED_ROTATED_RIGHT);
    public static final Pose2d RED_CENTER_ALLIANCE_SIDE_FACING_DOWNFIELD = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(1).get().getY() - NEUTRAL_ZONE_Y_OFFSET, RED_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d RED_LEFT_ALLIANCE_SIDE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_X_OFFSET),
            FIELD.getTagPose(6).get().getY() + NEUTRAL_ZONE_Y_OFFSET + Units.feetToMeters(2.0), RED_ROTATED_RIGHT);
    public static final Pose2d RED_HUB_SHOOTING = RED_IN_FRONT_OF_HUB_AUTO_LINE.plus(new Transform2d(-Units.feetToMeters(3.5), 0.0, RED_ROTATED_TOWARDS_DRIVERSTATION));
    public static final Pose2d RED_LEFT_UNDER_TRENCH_SHOOTING = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0) + Units.feetToMeters(0.5),
            Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_UNDER_TRENCH_SHOOTING = new Pose2d(FIELD.getTagPose(12).get().getX() + Units.inchesToMeters((47.0 / 2) - 13.0) + Units.feetToMeters(0.5),
            FIELD.getFieldWidth() - Units.feetToMeters(TRENCH_STARTING_OFFSET), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(12).get().getX() + BUMP_X_OFFSET_FROM_TRENCH + Units.feetToMeters(1.0),
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d RED_RIGHT_BUMP_SHOOTING_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(12).get().getX() + BUMP_X_OFFSET_FROM_TRENCH + Units.feetToMeters(3.0),
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d RED_LEFT_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(7).get().getX() + BUMP_X_OFFSET_FROM_TRENCH + Units.feetToMeters(3.0),
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);

    public static final Pose2d RED_RIGHT_CENTER_LINE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(1.5),
            FIELD.getTagPose(1).get().getY() - Units.feetToMeters(NEUTRAL_ZONE_INTERMEDIATE_Y_OFFSET), RED_ROTATED_LEFT);
    public static final Pose2d RED_LEFT_CENTER_LINE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(CENTER_LINE_INTERMEDIATE_X_OFFSET),
            FIELD.getTagPose(6).get().getY() + Units.feetToMeters(1.0), RED_ROTATED_RIGHT);
    public static final Pose2d RED_LEFT_CENTER_LINE_INTERMEDIATE_X_OFFSET = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(1.0),
            FIELD.getTagPose(6).get().getY() + Units.feetToMeters(1.5), RED_ROTATED_RIGHT);
    public static final Pose2d RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) - Units.feetToMeters(CENTER_LINE_INTERMEDIATE_X_OFFSET - 6.0) + Units.feetToMeters(1.5),
            FIELD.getTagPose(1).get().getY(), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(1).get().getY(), RED_ROTATED_LEFT);
    public static final Pose2d RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_RIGHT = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(1).get().getY() - Units.feetToMeters(2.0), RED_ROTATED_RIGHT);
    public static final Pose2d RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_DOWNFIELD = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(1).get().getY(), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET = new Pose2d((FIELD.getTagPose(1).get().getX() - Units.feetToMeters(5.0)),
            FIELD.getTagPose(1).get().getY(), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET_ROTATED_LEFT = new Pose2d((FIELD.getTagPose(1).get().getX() - Units.feetToMeters(5.0)),
            FIELD.getTagPose(1).get().getY(), RED_ROTATED_LEFT);
    public static final Pose2d RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET = new Pose2d((FIELD.getTagPose(1).get().getX() - Units.feetToMeters(5.0)),
            FIELD.getTagPose(6).get().getY(), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_ROTATED_RIGHT = new Pose2d((FIELD.getTagPose(1).get().getX() - Units.feetToMeters(5.0)),
            FIELD.getTagPose(6).get().getY(), RED_ROTATED_RIGHT);
    public static final Pose2d RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_Y_OFFSET = new Pose2d((FIELD.getTagPose(1).get().getX() - Units.feetToMeters(5.5)),
            FIELD.getTagPose(6).get().getY() + Units.feetToMeters(0.5), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(6).get().getY(), RED_ROTATED_DOWNFIELD);
    public static final Pose2d RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_LEFT = new Pose2d((FIELD.getFieldLength() / 2.0) + Units.feetToMeters(NEUTRAL_ZONE_ALLIANCE_SIDE_INTERMEDIATE_X_OFFSET - 2.0),
            FIELD.getTagPose(6).get().getY(), RED_ROTATED_LEFT);
    public static final Pose2d RED_LEFT_BUMP_NZ = new Pose2d(FIELD.getTagPose(6).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d RED_LEFT_BUMP_AS = new Pose2d(FIELD.getTagPose(7).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) - BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d RED_RIGHT_BUMP_NZ = new Pose2d(FIELD.getTagPose(1).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d RED_RIGHT_BUMP_AS = new Pose2d(FIELD.getTagPose(1).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_LEFT_45_DEGREES);
    public static final Pose2d RED_RIGHT_BUMP_NZ_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(1).get().getX() - BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);
    public static final Pose2d RED_RIGHT_BUMP_AS_ROTATED_RIGHT = new Pose2d(FIELD.getTagPose(1).get().getX() + BUMP_X_OFFSET_FROM_TRENCH,
            (FIELD.getFieldWidth() / 2.0) + BUMP_Y_OFFSET_FROM_CENTER, RED_ROTATED_RIGHT_45_DEGREES);

    // OUTPOST + DEPOT
    public static final Pose2d BLUE_OUTPOST_CENTERED = new Pose2d(FIELD.getTagPose(29).get().getX() + OUTPOST_OFFSET,
            FIELD.getTagPose(29).get().getY(), BLUE_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d BLUE_OUTPOST_INTERMEDIATE = new Pose2d(FIELD.getTagPose(29).get().getX() + Units.feetToMeters(6.0),
            FIELD.getTagPose(29).get().getY() + Units.feetToMeters(4.0), BLUE_ROTATED_LEFT);
    public static final Pose2d RED_OUTPOST_CENTERED = new Pose2d(FIELD.getTagPose(13).get().getX() - OUTPOST_OFFSET,
            FIELD.getTagPose(13).get().getY(), RED_ROTATED_TOWARDS_DRIVERSTATION);
    public static final Pose2d RED_OUTPOST_INTERMEDIATE = new Pose2d(FIELD.getTagPose(13).get().getX() - Units.feetToMeters(6.0),
            FIELD.getTagPose(13).get().getY() - Units.feetToMeters(4.0), RED_ROTATED_LEFT);

    public static final Pose2d BLUE_BEHIND_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(23).get().getX() - Units.inchesToMeters(36.5) - Units.feetToMeters(1.5),
            FIELD.getFieldWidth() / 2.0 + DEPOT_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_DOWNFIELD);
    public static final Pose2d RED_BEHIND_BUMP_SHOOTING = new Pose2d(FIELD.getTagPose(7).get().getX() + Units.inchesToMeters(36.5) + Units.feetToMeters(1.5),
            FIELD.getFieldWidth() / 2.0 - DEPOT_Y_OFFSET_FROM_CENTER, RED_ROTATED_DOWNFIELD);
    public static final Pose2d BLUE_DEPOT_INTERMEDIATE = new Pose2d(DEPOT_X_OFFSET,
            FIELD.getFieldWidth() / 2.0 + DEPOT_Y_OFFSET_FROM_CENTER, BLUE_ROTATED_LEFT);
    public static final Pose2d RED_DEPOT_INTERMEDIATE = new Pose2d(FIELD.getFieldLength() - DEPOT_X_OFFSET,
            FIELD.getFieldWidth() / 2.0 - DEPOT_Y_OFFSET_FROM_CENTER, RED_ROTATED_LEFT);
    public static final Pose2d BLUE_DEPOT = new Pose2d(DEPOT_X_OFFSET,
            FIELD.getFieldWidth() / 2.0 + DEPOT_Y_OFFSET_FROM_CENTER + Units.inchesToMeters(42.0), BLUE_ROTATED_LEFT);
    public static final Pose2d RED_DEPOT = new Pose2d(FIELD.getFieldLength() - DEPOT_X_OFFSET,
            FIELD.getFieldWidth() / 2.0 - DEPOT_Y_OFFSET_FROM_CENTER - Units.inchesToMeters(42.0), RED_ROTATED_LEFT);

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
        // The Pose2d's of the six locations we will want to shoot fuel to
        public static final Translation2d BLUE_HUB = new Translation2d(
                ((FIELD.getTagPose(26).get().getX() + FIELD.getTagPose(20).get().getX()) / 2.0),
                ((FIELD.getTagPose(18).get().getY() + FIELD.getTagPose(21).get().getY()) / 2.0)
        );
        public static final Translation2d RED_HUB = new Translation2d(
                ((FIELD.getTagPose(10).get().getX() + FIELD.getTagPose(4).get().getX()) / 2.0),
                ((FIELD.getTagPose(2).get().getY() + FIELD.getTagPose(8).get().getY()) / 2.0)
        );


        public static final Translation2d PASS_DEPOT_BLUE = new Translation2d(
                (FIELD.getTagPose(29).get().getX() + 0.5),
                (FIELD.getTagPose(23).get().getY() - 0.5 - Units.feetToMeters(2.5)));
        public static final Translation2d PASS_OUTPOST_BLUE = new Translation2d(
                (FIELD.getTagPose(29).get().getX() + 0.5),
                (FIELD.getTagPose(29).get().getY() + 0.5 + Units.feetToMeters(2.5)));
        public static final Translation2d PASS_OUTPOST_RED = new Translation2d(
                (FIELD.getTagPose(13).get().getX() - 0.5),
                (FIELD.getTagPose(13).get().getY() - 0.5 - Units.feetToMeters(2.5)));
        public static final Translation2d PASS_DEPOT_RED = new Translation2d(
                (FIELD.getTagPose(13).get().getX() - 0.5),
                (FIELD.getTagPose(7).get().getY() + 0.5 + Units.feetToMeters(2.5)));

        // The Translation the turret is from the center of the robot
        public static final Translation2d TURRET_OFF_CENTER = new Translation2d(Units.inchesToMeters(-5.5), Units.inchesToMeters(7.25));

        // The velocity of the Shooter while not shooting
        public static final double DEFAULT_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(1000.0);
    }

    public static void RecordOutputs() {
        Logger.recordOutput("Field/BLUE_LEFT_CENTER_LINE_INTERMEDIATE", RED_LEFT_CENTER_LINE_INTERMEDIATE);
        Logger.recordOutput("Field/BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE", RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE);
        Logger.recordOutput("Field/BLUE_RIGHT_CENTER_LINE_INTERMEDIATE", RED_RIGHT_CENTER_LINE_INTERMEDIATE);
        Logger.recordOutput("Field/BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE", BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE);
        Logger.recordOutput("Field/RedOutpostCentered", RED_HUB_SHOOTING);
    }
}
