package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

public class AutoPaths {
    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;
    private final CommandFactory commandFactory;

    private Pose2d goToPositionTarget;

    // The distance the robot has to be at before the goToPosition command switches from the
    //     intermediate target to the actual target
    private final double DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD = Units.feetToMeters(2.0);
    // Default distance for how close the the trench poses the robot can get before switching targets
    private final double DEFAULT_TRENCH_DISTANCE_THRESHOLD = Units.feetToMeters(0.5);
    // How long the robot should sit in place to shoot under the trench
    private final double DEFAULT_SHOOTING_TIME = 6.0;

    public AutoPaths(Drive drive, Intake intake, Shooter shooter, CommandFactory commandFactory) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
        this.commandFactory = commandFactory;
    }

    /**
     * Returns the pose of the correct alliance
     *
     * @param blueTargetPose the pose to return when the robot is on the blue alliance
     * @param redTargetPose  the pose to return when the robot is on the red alliance
     */
    private Pose2d alliancePose(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue ? blueTargetPose : redTargetPose;
    }

    /**
     * Resetting the odometry of the drivetrain based on the robot's alliance
     * <p>
     * requires the {@link Drive} subsystem and executes instantly
     * </p>
     *
     * @param blueTargetPose the pose to reset the odometry to if on the blue alliance
     * @param redTargetPose  the pose to reset the odometry to if on the red alliance
     */
    private Command resetOdometry(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return Commands.defer(() -> drive.resetOdometry(() -> alliancePose(blueTargetPose, redTargetPose)), Set.of(drive));
    }

    private Command zeroIntakeAndHood() {
//        return intake.zeroPivot().alongWith(shooter.zeroHood());
        return shooter.zeroHood();
    }

    /**
     * Drives the robot to desired positions, using intermediate positions if supplied
     * <p>
     * this command requires {@link Drive} and does not end
     * </p>
     * <p>
     * If intermediate positions are supplied, the robot will attempt to drive toward those instead of the target position.
     * * If the intermediate position supplier has no more intermediate positions, or no longer wishes the for the robot to go to an intermediate position, it should return {@code null}.
     * * Once a supplier has returned {@code null}, it will no longer be checked for intermediate positions and the robot will drive directly to the target position.
     * </p>
     *
     * @param targetPose               the target position of the robot
     * @param intermediatePoseSupplier a supplier that is possibly {@code null} of intermediate positions to travel to
     * @see #goToPosition(Pose2d, Pose2d)
     */
    private Command goToPosition(Supplier<Pose2d> targetPose, Supplier<Pose2d> intermediatePoseSupplier, BooleanSupplier slowVelocity) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};
        Pose2d[] startingPose = new Pose2d[]{null};

        return drive.goToPosition(() -> {
                            Pose2d target;
                            if (intermediateHolder[0] != null && ((intermediateHolder[0] = intermediatePoseSupplier.get()) != null))
                                target = intermediateHolder[0];
                            else
                                target = targetPose.get();

                            goToPositionTarget = target;
                            return goToPositionTarget;
                        },
                        () -> false,
                        () -> (slowVelocity.getAsBoolean() ? drive.getMaxLinearSpeedMetersPerSec() * 0.5 : drive.getMaxLinearSpeedMetersPerSec()))
                .beforeStarting(() -> {
                    if (intermediatePoseSupplier != null) intermediateHolder[0] = intermediatePoseSupplier.get();
                    else Commands.none();
                    startingPose[0] = drive.getPose();
                });
    }

    /**
     * Drives the robot to the target position
     * <p>
     * This command requires {@link Drive} and does not end
     * </p>
     *
     * @param blueTargetPose target position of the robot if on blue alliance
     * @param redTargetPose  target position of the robot if on red alliance
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPosition(() -> alliancePose(blueTargetPose, redTargetPose), () -> null, () -> false)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPositionSlowAccel(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPosition(()-> alliancePose(blueTargetPose, redTargetPose), ()-> null, ()->true)
                .until(()-> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot to a target position
     * <p> This command uses the {@link #DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD} to determine the threshold distance for the
     * robot to switch to the target</p>
     * <p>
     * This command requires {@link Drive} and ends when the robot is at the target position
     * </p>
     *
     * @param blueTargetPose                   Target position of the robot if on the blue alliance
     * @param redTargetPose                    Target position of the robot if on the red alliance
     * @param blueAllianceIntermediateSupplier A supplier of positions the robot should go to if on the blue alliance
     * @param redAllianceIntermediateSupplier  A supplier of positions the robot should go to if on the red alliance
     * @param angleDistanceThreshold           At what point during the path the robot should be finished rotating
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueAllianceIntermediateSupplier, Supplier<Pose2d> redAllianceIntermediateSupplier, double angleDistanceThreshold, BooleanSupplier slowVelocity) {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};

        return goToPosition(
                () -> alliancePose(blueTargetPose, redTargetPose),
                () -> {
                    if (intermediatePoseHolder[0] != null) {
                        if (!drive.atPosition(intermediatePoseHolder[0].getTranslation(), DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD))
                            return intermediatePoseHolder[0];
                        else return null;
                    } else return null;
                },
                () -> slowVelocity.getAsBoolean())
                .beforeStarting(() -> {
                    if (blueAllianceIntermediateSupplier != null)
                        intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediateSupplier.get(), redAllianceIntermediateSupplier.get());
                    else Commands.none();
                })
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot towards a target position until it reaches a certain threshold away
     * <p> This command requires {@link Drive} and ends when the robot reaches a threshold distance away from the target</p>
     *
     * @param blueTargetPose Target position of the robot if on the blue alliance
     * @param redTargetPose  Target position of the robot if on the red alliance
     * @param threshold      At what distance away from the target the robot ends the command
     */
    private Command goToIntermediate(Pose2d blueTargetPose, Pose2d redTargetPose, double threshold) {
        return goToPosition(blueTargetPose, redTargetPose)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(), threshold));
    }

    private Command goToIntermediateSlowAccel(Pose2d blueTargetPose, Pose2d redTargetPose, double threshold) {
        return goToPositionSlowAccel(blueTargetPose, redTargetPose)
                .until(()-> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(), threshold));
    }

    /**
     * Deploys and runs the intake
     * <p> This command requires {@link Intake}</p>
     */
    private Command intaking() {
        return intake.deployAndIntake();
    }

    /**
     * Runs the shooter and feeding system
     * <p>
     * This command requires {@link Shooter}, {@link Intake}
     *
     * @param pivotAngle The angle the intake should be at while feeding
     */
    private Command shooting(double pivotAngle) {
        return commandFactory.autonomousFeedAndShoot(true, pivotAngle)
                .withTimeout(DEFAULT_SHOOTING_TIME)
                .andThen(shooter.stop());
    }

    /**
     * This command runs the shooter and feeding system with the intake at a 130 degree angle
     * <p> This command requires {@link Shooter}, {@link Intake}</p>
     * <p> This command uese {@link #shooting(double)}</p>
     */
    private Command shooting() {
        return shooting(Units.degreesToRadians(130.0));
    }

    // Auto Routines
    public Command rightStarting_neutralZone_shoot_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(3.0)),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE, Units.feetToMeters(4.0))
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                goToPosition(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, Units.feetToMeters(4.0)),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_TRENCH_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(), runOnce(drive::stop)),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_RIGHT_CENTER_ALLIANCE_SIDE, Constants.RED_RIGHT_CENTER_ALLIANCE_SIDE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_DOWNFIELD, Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_DOWNFIELD, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_TRENCH_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING)
                        .raceWith(commandFactory.shooterDefault(()-> true)),
                Commands.parallel(shooting(), runOnce(drive::stop)));
    }

    public Command leftStarting_neutralZone_shoot_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(3.0)),
                goToIntermediate(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE, Units.feetToMeters(4.0))
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                goToPosition(Constants.BLUE_LEFT_CENTER_LINE, Constants.RED_LEFT_CENTER_LINE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE, Units.feetToMeters(4.0)),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_TRENCH_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(), runOnce(drive::stop)),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_LEFT_ALLIANCE_SIDE, Constants.RED_LEFT_ALLIANCE_SIDE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE, DEFAULT_TRENCH_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING)
                        .raceWith(commandFactory.shooterDefault(()-> true)),
                Commands.parallel(shooting(), runOnce(drive::stop)));
    }

    public Command leftStarting_neutralZone_shoot_neutralZoneToRightSide() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_Y_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET_Y_OFFSET, Units.feetToMeters(3.0)),
                goToIntermediate(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE_X_OFFSET, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE_X_OFFSET, Units.feetToMeters(4.0))
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                goToPositionSlowAccel(Constants.BLUE_LEFT_CENTER_LINE, Constants.RED_LEFT_CENTER_LINE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(1.5)),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(), runOnce(drive::stop)),
                goToIntermediate(Constants.BLUE_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_LEFT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Units.feetToMeters(4.0))
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                goToIntermediateSlowAccel(Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_RIGHT, Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_ROTATED_RIGHT, Units.feetToMeters(4.0))
                        .raceWith(intaking()),
                goToIntermediateSlowAccel(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(1.5))
                        .raceWith(intaking()),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING)
                        .raceWith(shooting()),
                Commands.parallel(shooting(), runOnce(drive::stop))
        );
    }

    public Command rightStarting_neutralZone_shoot_neutralZoneLoop_shoot() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(3.0)),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE, Units.feetToMeters(4.0))
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                goToPosition(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, Units.feetToMeters(4.0)),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_TRENCH_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(), runOnce(drive::stop)),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Units.feetToMeters(3.0)),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD)
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_CENTER_ALLIANCE_SIDE_FACING_UPFIELD, Constants.RED_CENTER_ALLIANCE_SIDE_FACING_DOWNFIELD, Units.feetToMeters(4.0))
                        .raceWith(intaking()),
                goToIntermediate(Constants.BLUE_RIGHT_CENTER_ALLIANCE_SIDE_Y_OFFSET, Constants.RED_RIGHT_CENTER_ALLIANCE_SIDE_Y_OFFSET, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, Constants.RED_RIGHT_NEUTRAL_ZONE_TRENCH_OFFSET, DEFAULT_TRENCH_DISTANCE_THRESHOLD)
                        .raceWith(intaking()),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING)
                        .raceWith(shooting())
        );
    }

    public Command rightStarting_outpost() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                goToIntermediate(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_OUTPOST_INTERMEDIATE, Constants.RED_OUTPOST_INTERMEDIATE, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToPosition(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED)
                        .raceWith(shooting())
        );
    }

    public Command middleStarting_depot() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_IN_FRONT_OF_HUB_AUTO_LINE, Constants.RED_IN_FRONT_OF_HUB_AUTO_LINE),
                goToIntermediate(Constants.BLUE_HUB_SHOOTING, Constants.RED_HUB_SHOOTING, DEFAULT_INTERMEDIATE_DISTANCE_THRESHOLD),
                goToIntermediate(Constants.BLUE_DEPOT_INTERMEDIATE, Constants.RED_DEPOT_INTERMEDIATE, DEFAULT_TRENCH_DISTANCE_THRESHOLD)
                        .raceWith(intaking()),
                goToPosition(Constants.BLUE_DEPOT, Constants.RED_DEPOT)
                        .raceWith(intaking()),
                shooting()
                        .raceWith(Commands.runOnce(drive::stop))
        );
    }

    public Command middleStarting_shootIntoHub() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_IN_FRONT_OF_HUB_AUTO_LINE, Constants.RED_IN_FRONT_OF_HUB_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPosition(Constants.BLUE_HUB_SHOOTING, Constants.RED_HUB_SHOOTING),
                shooting(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        );
    }

    public Command leftStarting_shootIntoHub() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                shooting(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        );
    }

    public AutonomousMode rightStarting_shootIntoHub() {
        return new AutonomousMode(
                Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE,
                Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                shooting(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        ));
    }

    public record AutonomousMode(
            Pose2d initialRedPosition,
            Pose2d initialBluePosition,
            Command command
    ) {}
}
