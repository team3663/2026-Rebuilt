package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
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
    private final double INTERMEDIATE_DISTANCE_THRESHOLD = 4.0;
    // The distance at which the robot stops the goTowardsIntermediate command and starts going to next position
    private final double LINKING_PATHS_DISTANCE_THRESHOLD = 2.0;
    // At what point the drivetrain should finish rotating, currently set to the end of the path
    private final double DEFAULT_ANGLE_DISTANCE_THRESHOLD = 3.0;
    // How long the robot should sit in place to shoot under the trench
    private final double DEFAULT_SHOOTING_TIME = 6.0;
    // How long the robot should sit in place to shoot at the outpost
    private final double OUTPOST_SHOOTING_TIME = 5.0;

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

    private double getDistanceToPose(Pose2d bluePose, Pose2d redPose) {
        return (Units.feetToMeters(drive.getPose().getTranslation().getDistance(alliancePose(bluePose, redPose).getTranslation()))
                + Units.feetToMeters(3.0));
    }

    // TODO implement this
    private Command zeroIntakeAndHood() {
        return intake.zeroPivot().alongWith(shooter.zeroHood());
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
     * @see #goToPosition(Pose2d, double)
     */
    private Command goToPosition(Supplier<Pose2d> targetPose, Supplier<Pose2d> intermediatePoseSupplier, BooleanSupplier slowVelocity, double angleDistanceThreshold) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};
        Pose2d[] startingPose = new Pose2d[]{null};

        return drive.goToPosition(() -> {
                            Pose2d target;
                            if (intermediateHolder[0] != null && ((intermediateHolder[0] = intermediatePoseSupplier.get()) != null))
                                target = intermediateHolder[0];
                            else
                                target = targetPose.get();

                            var distanceStartToTarget = startingPose[0].getTranslation().getDistance(target.getTranslation());
                            var distanceCurrentToTarget = drive.getPose().getTranslation().getDistance(target.getTranslation());
                            var t = MathUtil.inverseInterpolate(angleDistanceThreshold, distanceStartToTarget, distanceCurrentToTarget);

                            goToPositionTarget = target;
                            return goToPositionTarget;
//                            return new Pose2d(goToPositionTarget.getTranslation(),
//                                    target.getRotation().interpolate(startingPose[0].getRotation(), t));
                        },
                        ()-> false,
                        () -> (slowVelocity.getAsBoolean() ? drive.getMaxLinearSpeedMetersPerSec() * 0.5 : drive.getMaxLinearSpeedMetersPerSec()))
                .beforeStarting(() -> {
                    if (intermediatePoseSupplier != null) intermediateHolder[0] = intermediatePoseSupplier.get();
                    else Commands.none();
                    startingPose[0] = drive.getPose();
                });
    }

    /**
     * Drives the robot to the target position
     * <p>This command uses the {@link #DEFAULT_ANGLE_DISTANCE_THRESHOLD} to determine when the robot should be finished
     * * rotating</p>
     * <p>
     * this command requires {@link Drive} and does not end
     * </p>
     *
     * @param targetPose the target position of the robot
     * @see #goToPosition(Pose2d, Pose2d)
     * @see #goToPosition(Supplier, Supplier, BooleanSupplier, double)
     */
    private Command goToPosition(Pose2d targetPose, double angleDistanceThreshold) {
        return goToPosition(() -> targetPose, () -> null, () -> false, angleDistanceThreshold);
    }

    /**
     * Drives the robot to the target position
     * <p>This command uses the {@link #DEFAULT_ANGLE_DISTANCE_THRESHOLD} to determine when the robot should be finished
     * rotating</p>
     * <p>
     * This command requires {@link Drive} and does not end
     * </p>
     *
     * @param blueTargetPose target position of the robot if on blue alliance
     * @param redTargetPose  target position of the robot if on red alliance
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPosition(() -> alliancePose(blueTargetPose, redTargetPose), () -> null, () -> false, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD))
                .until(()-> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot to a target position
     * <p> This command uses the {@link #INTERMEDIATE_DISTANCE_THRESHOLD} to determine the threshold distance for the
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
                        if (!drive.atPosition(intermediatePoseHolder[0].getTranslation(), Units.feetToMeters(INTERMEDIATE_DISTANCE_THRESHOLD)))
                            return intermediatePoseHolder[0];
                        else return null;
                    } else return null;
                },
                () -> slowVelocity.getAsBoolean(),
                angleDistanceThreshold)
                .beforeStarting(() -> {
                    if (blueAllianceIntermediateSupplier != null)
                        intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediateSupplier.get(), redAllianceIntermediateSupplier.get());
                    else Commands.none();
                })
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueAllianceIntermediateSupplier, Supplier<Pose2d> redAllianceIntermediateSupplier, double angleDistanceThreshold){
        return goToPosition(blueTargetPose, redTargetPose, blueAllianceIntermediateSupplier, redAllianceIntermediateSupplier, angleDistanceThreshold, ()-> false);
    }

    /**
     * Drives the robot to a target position
     * <p> This command uses the {@link #DEFAULT_ANGLE_DISTANCE_THRESHOLD} to determine the threshold distance for when
     * in the path the robot should finish rotating</p>
     * <p>Runs the shooter at its default speed and turret angle</p>
     * <p>
     * This command requires {@link Drive}, {@link Shooter}, {@link Intake} and ends when the robot is at the target position
     * </p>
     *
     * @param blueTargetPose       Target position of the robot if on the blue alliance
     * @param redTargetPose        Target position of the robot if on the red alliance
     * @param blueIntermediatePose A supplier of positions the robot should go to if on the blue alliance
     * @param redIntermediatePose  A supplier of positions the robot should go to if on the red alliance
     * @param shouldZero           A boolean for if the command should zero the intake pivot and hood
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose, boolean shouldZero) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD))
                .alongWith(commandFactory.shooterDefault())
                .beforeStarting(shouldZero ? intake.zeroPivot().alongWith(shooter.zeroHood()) : Commands.none())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPositionJustDriving(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                            Supplier<Pose2d> redIntermediatePose, double angleDistance, BooleanSupplier slowVelocity) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, angleDistance, slowVelocity)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));

    }

    private Command goToPositionJustDriving(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                            Supplier<Pose2d> redIntermediatePose, double angleDistance) {
        return goToPositionJustDriving(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, angleDistance, ()-> false);

    }

    /**
     * Drives the robot to a target position
     * This commands uses the {@link Drive} subsystem
     *
     * @param blueTargetPose       Target position if on the blue alliance
     * @param redTargetPose        Target position if on the red alliance
     * @param blueIntermediatePose A supplier of positions for the robot to go to if on the blue alliance
     * @param redIntermediatePose  A supplier of positions for the robot to go to if on the red alliance
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                 Supplier<Pose2d> redIntermediatePose) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, false);

    }

    /**
     * Drives the robot towards a given target until the robot is within a certain distance of it's target
     * <p>This command uses {@link Drive}</p>
     * <p>This command uses the {@link #LINKING_PATHS_DISTANCE_THRESHOLD} to determine the distance away
     * from the target at which the command ends</p>
     *
     * @param blueTargetPose Target position to drive towards if on the blue alliance
     * @param redTargetPose  Target position to drive towards if on the red alliance
     */
    private Command goTowardsIntermediate(Pose2d blueTargetPose, Pose2d redTargetPose,
                                          Supplier<Pose2d> blueIntermediatePositon, Supplier<Pose2d> redIntermediatePosition, double angleDistance) {
        return goToPositionJustDriving(blueTargetPose, redTargetPose, blueIntermediatePositon, redIntermediatePosition, angleDistance)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose)
                        .getTranslation(), Units.feetToMeters(LINKING_PATHS_DISTANCE_THRESHOLD)));
    }

    private Command goTowardsIntermediate(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goTowardsIntermediate(blueTargetPose, redTargetPose, () -> null, () -> null, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Drive}, {@link Intake} and ends when the robot reaches the target position
     *
     * @param blueTargetPose               target position of the robot if on blue alliance
     * @param redTargetPose                target position of the robot if on red alliance
     * @param blueIntermediatePoseSupplier a supplier that is possibly {@code null} of intermediate positions to travel to if on blue alliance
     * @param redIntermediatePoseSupplier  a supplier that is possibly {@code null} of intermediate positions to travel to if on red alliance
     * @see #intaking(Pose2d, Pose2d) for an option with no intermediate suppliers
     */
    private Command intaking(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePoseSupplier, Supplier<Pose2d> redIntermediatePoseSupplier, double angleDistance) {
        return goToPositionJustDriving(blueTargetPose, redTargetPose,
                blueIntermediatePoseSupplier, redIntermediatePoseSupplier, angleDistance, ()-> true)
                .alongWith(intake.deployAndIntake())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command intaking(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediateSupplier, Supplier<Pose2d> redIntermediateSupplier) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediateSupplier, redIntermediateSupplier, 0.0);
    }

    private Command intaking(Pose2d blueTaretPose, Pose2d redTargetPose, double angleDistance) {
        return goToPosition(blueTaretPose, redTargetPose, () -> null, () -> null, angleDistance)
                .alongWith(intake.deployAndIntake())
                .until(() -> drive.atPosition(alliancePose(blueTaretPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot to a target position while intaking
     * This command zeros the intake pivot and hood
     * This command uses {@link Drive}, {@link Intake}, {@link Shooter} subsystems
     *
     * @param blueTargetPose               Target position of the robot if on the blue alliance
     * @param redTargetPose                Target position of the robot if on the red alliance
     * @param blueIntermediatePoseSupplier A supplier of positions for the robot to go to if on the blue alliance
     * @param redIntermediatePoseSupplier  A supplier of positions for the robot to go to if on the red alliance
     */
    private Command intakingAndZeroing(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePoseSupplier, Supplier<Pose2d> redIntermediatePoseSupplier, double angleDistance) {
        return Commands.parallel(goToPositionJustDriving(blueTargetPose, redTargetPose,
                                blueIntermediatePoseSupplier, redIntermediatePoseSupplier, Units.feetToMeters(angleDistance), ()-> true),
                        intake.zeroPivot().alongWith(shooter.zeroHood())
                                .andThen(intake.deployAndIntake()))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command intakingAndZeroing(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                       Supplier<Pose2d> redIntermediatePose) {
        return intakingAndZeroing(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * <p>
     * This command uses {@link Drive}, {@link Intake}
     * </p>
     * This command is an alternative to {@link #intaking(Pose2d, Pose2d, Supplier, Supplier)}
     *
     * @param blueTargetPose target position of the robot if on blue alliance
     * @param redTargetPose  target position of the robot if on red alliance
     * @see #intaking(Pose2d, Pose2d, Supplier, Supplier)
     */
    private Command intaking(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intaking(blueTargetPose, redTargetPose, () -> null, () -> null);
    }

    private Command intakingWithDistanceThreshold(Pose2d blueTargetPose, Pose2d redTargetPose,
                                                  Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose, double angleDistance) {
        return goTowardsIntermediate(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, angleDistance)
                .alongWith(intake.deployAndIntake())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(), Units.feetToMeters(LINKING_PATHS_DISTANCE_THRESHOLD)));
    }

    private Command intakingWithDistanceThreshold(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                                  Supplier<Pose2d> redIntermediatePose) {
        return intakingWithDistanceThreshold(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    private Command intakingWithDistanceThreshold(Pose2d blueTargetPose, Pose2d redTargetPose, double angleDistance) {
        return intakingWithDistanceThreshold(blueTargetPose, redTargetPose, () -> null, () -> null, angleDistance);
    }

    private Command intakingWithDistanceThreshold(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intakingWithDistanceThreshold(blueTargetPose, redTargetPose, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    /**
     * Runs the shooter in place
     * <p>
     * This command requires {@link Shooter}, {@link Intake}
     *
     * @param timeout Time in <b>seconds</b> for how long to let the robot shoot for
     */
    private Command shooting(double timeout, double pivotAngle) {
        return commandFactory.autonomousFeedAndShoot(true, pivotAngle)
                .withTimeout(timeout)
                .andThen(shooter.stop());
    }

    private Command shooting(double timeout) {
        return shooting(timeout, Units.degreesToRadians(70.0));
    }

    /**
     * Runs the shooter in place
     * This command uses {@link Shooter}
     */
    private Command shootingInPlace() {
        return shooting(DEFAULT_SHOOTING_TIME);
    }

    private Command shootingInPlace(double angle) {
        return shooting(DEFAULT_SHOOTING_TIME, angle);
    }

    /**
     * Runs the shooter in place and zeros the intake pivot and hood
     * <p>
     * This command uses {@link Shooter}, {@link Intake}
     * <p>
     * This command uses {@link #DEFAULT_SHOOTING_TIME} to determine how long to shoot for
     */
    private Command zeroAndShootInPlace() {
        return commandFactory.autonomousFeedShootAndZero(true).withTimeout(DEFAULT_SHOOTING_TIME);
    }

    /**
     * This command drives the robot to a target position while shooting until at the target position
     * <p> This command uses {@link Drive}, {@link Shooter}, {@link Intake}</p>
     *
     * @param blueTargetPose       The target position if the robot is on the blue alliance
     * @param redTargetPose        The target position if the robot is on the red alliance
     * @param blueIntermediatePose A supplier of positions for the robot to drive to if on the blue alliance
     * @param redIntermediatePose  A supplier of positions for the robot to drive to if on the red alliance
     */
    private Command goToPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                         Supplier<Pose2d> redIntermediatePose, double angleDistance) {
        return goToPositionJustDriving(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, angleDistance)
                .alongWith(commandFactory.autonomousFeedAndShoot(true, Units.degreesToRadians(70.0)))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                         Supplier<Pose2d> redIntermediatePose) {
        return goToPositionAndShoot(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    /**
     * Drives the robot to a target position while shooting and zeros the intake pivot and hood
     * <p> This command uses {@link Drive}, {@link Shooter}, {@link Intake}</p>
     *
     * @param blueTargetPose Target pose for the robot to drive to if on the blue alliance
     * @param redTargetPose  Target pose for the robot to drive to if on the red alliance
     */
    private Command goToPositionShootAndZero(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPositionJustDriving(blueTargetPose, redTargetPose, () -> null, () -> null, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD))
                .alongWith(commandFactory.autonomousFeedShootAndZero(true))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot to a target position while shooting
     * <p> This command uses {@link Drive}, {@link Shooter}</p>
     * <p>
     * </p>
     *
     * @param blueTargetPose Target pose for the robot to drive to if on the blue alliance
     * @param redTargetPose  Target pose for the robot to drive to if on the red alliance
     */
    private Command goToPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPositionAndShoot(blueTargetPose, redTargetPose, () -> null, () -> null, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD));
    }

    private Command goTowardsPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose,
                                              double angleDistance, double shootingTime) {
        return goTowardsIntermediate(blueTargetPose, redTargetPose, () -> null, () -> null, angleDistance)
                .alongWith(shooting(shootingTime))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(),
                        Units.feetToMeters(LINKING_PATHS_DISTANCE_THRESHOLD)));
    }

    /**
     * Drives to a target position while intaking and shooting until the robot reaches the target position
     * <p> This command requires {@link Drive}, {@link Intake}, {@link Shooter}</p>
     * <p>This command uses {@link #intaking(Pose2d, Pose2d, Supplier, Supplier)}</p>
     *
     * @param blueTargetPose       Target pose for the robot to drive to if on the blue alliance
     * @param redTargetPose        Target pose for the robot to drive to if on the red alliance
     * @param blueIntermediatePose A supplier of positions for the robot to drive to if on the blue alliance
     * @param redIntermediatePose  A supplier of positions for the robot to drive to if on the red alliance
     */
    private Command intakeAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose,
                                   Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(shooting(DEFAULT_SHOOTING_TIME))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot while intaking and shooting until the robot reaches the target position
     * <p>This command uses {@link Drive}, {@link Shooter}, {@link Intake}</p>
     * <p>This command uses {@link #intakeAndShoot(Pose2d, Pose2d, Supplier, Supplier)}</p>
     *
     * @param blueTargetPose Target pose for the robot to drive to if on the blue alliance
     * @param redTargetPose  Target pose for the robot to drive to if on the red alliance
     */
    private Command intakeAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intakeAndShoot(blueTargetPose, redTargetPose, () -> null, () -> null);
    }

    /**
     * Drives the robot while intaking and passing until the robot reaches the target position
     * <p>This command uses {@link Drive}, {@link Shooter}</p>
     * <p>This command uses {@link #intaking(Pose2d, Pose2d, Supplier, Supplier)}</p>
     *
     * @param blueTargetPose       Target pose for the robot to drive to if on the blue alliance
     * @param redTargetPose        Target pose for the robot to drive to if on the red alliance
     * @param blueIntermediatePose Supplier of positions for the robot to go to if on the blue alliance
     * @param redIntermediatePose  Supplier of positions for the robot to go to if on the red alliance
     */
    private Command intakeAndPass(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(commandFactory.aim(false))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command intakeAndPassTowardsPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                                 Supplier<Pose2d> redIntermediatePose, double angleDistance) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, angleDistance)
                .alongWith(commandFactory.autonomousFeedAndShootWithoutIntake(false, false))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(), Units.feetToMeters(LINKING_PATHS_DISTANCE_THRESHOLD)));
    }

    /**
     * Drives the robot to a target position while intaking, passing, and zeroing the intake pivot and hood.
     * This command ends when the robot reaches its target position.
     * <p> This command uses {@link Drive}, {@link Shooter}, {@link Intake}</p>
     * <p>This command uses {@link #intaking(Pose2d, Pose2d, Supplier, Supplier)}</p>
     *
     * @param blueTargetPose       Target pose of the robot if on the blue alliance
     * @param redTargetPose        Target pose of the robot if on the blue alliance
     * @param blueIntermediatePose Supplier of positions for the robot to go to if on the blue alliance
     * @param redIntermediatePose  Supplier of positions for the robot to go to if on the red alliance
     */
    private Command intakePassAndZero(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                      Supplier<Pose2d> redIntermediatePose) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(commandFactory.aim(false))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()))
                .beforeStarting(intake.zeroPivot().alongWith(shooter.zeroHood()));
    }

    private Command intakePassAndZeroTowardsPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose,
                                                     Supplier<Pose2d> redIntermediatePose, boolean stopShooter, double linkingThreshold) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(commandFactory.aim(false))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation(),
                        Units.feetToMeters(linkingThreshold)))
                .beforeStarting(intake.zeroPivot().alongWith(shooter.zeroHood()));
    }

    private Command intakePassAndZeroTowardsPosition(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intakePassAndZeroTowardsPosition(blueTargetPose, redTargetPose, () -> null, () -> null, true, LINKING_PATHS_DISTANCE_THRESHOLD);
    }

    /**
     * Drives the robot to a target position and then climbs
     * <p>This command uses {@link Drive}, {@link Intake}</p>
     * <This command uses {@link #goToPosition(Pose2d, Pose2d, Supplier, Supplier, boolean)}
     *
     * @param blueTargetPose       Target pose of the robot if on the blue alliance
     * @param redTargetPose        Target pose of the robot if on the red alliance
     * @param blueIntermediatePose Supplier of poses for the robot to go to if on the blue alliance
     * @param redIntermediatePose  Supplier of poses for the robot to go to if on the red alliance
     */
    //TODO add climbing command when climber is added to main
    private Command goToPositionAndClimb(Pose2d blueTargetPose, Pose2d redTargetPose,
                                         Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose, false)
                // TODO make this not run until the robot is almost at the target climb positiom
                .alongWith(intake.stow());
    }

    /**
     * Drives the robot to a target position and then climbs
     * <p> This command uses {@link Drive}, {@link Intake}</p>
     * <p>This command uses {@link #goToPositionAndClimb(Pose2d, Pose2d, Supplier, Supplier)}</p>
     *
     * @param blueTargetPose Target pose for the robot to go to if on the blue alliance
     * @param redTargetPose  Target pose for the robot to go to if on the red alliance
     */
    //TODO add climbing command when climber is added to main
    private Command goToPositionAndClimb(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPositionAndClimb(blueTargetPose, redTargetPose, () -> null, () -> null)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    // Auto Routines
    public Command leftStarting_neutralZone_neutralZoneSwoop() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intaking(Constants.BLUE_LEFT_CENTER_LINE, Constants.RED_LEFT_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE,
                        Units.feetToMeters(3.0)),
                shootingInPlace(),
                intakingWithDistanceThreshold(Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intakingWithDistanceThreshold(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_LEFT,
                        getDistanceToPose(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_LEFT)),
                intakingWithDistanceThreshold(Constants.BLUE_MIDDLE_CENTER_LINE_ROTATED_LEFT, Constants.RED_MIDDLE_CENTER_LINE_ROTATED_LEFT,
                        Units.feetToMeters(3.0)),
                intakingWithDistanceThreshold(Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Units.feetToMeters(3.0)),
                shootingInPlace()
        );
    }

    public Command rightStarting_neutralZone_outpost() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakingAndZeroing(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE,
                        5.0),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE,
                        Units.feetToMeters(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE.getX()) + 5.0),
                goToPositionAndShoot(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED,
                        () -> Constants.BLUE_OUTPOST_INTERMEDIATE, () -> Constants.RED_OUTPOST_INTERMEDIATE, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD)),
                Commands.parallel(shootingInPlace(), runOnce(drive::stop)),
                goTowardsIntermediate(Constants.BLUE_OUTPOST_INTERMEDIATE, Constants.RED_OUTPOST_INTERMEDIATE),
                goTowardsPositionAndShoot(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        Units.feetToMeters(5.0), OUTPOST_SHOOTING_TIME),
                intaking(Constants.BLUE_RIGHT_ALLIANCE_SIDE, Constants.RED_RIGHT_ALLIANCE_SIDE,
                        ()-> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, ()-> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE)
        );
    }

    public Command rightStarting_neutralZone_shoot_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                goToPosition(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE)
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                intaking(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE, ()-> null, ()-> null),
                goToPosition(Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(5.0), runOnce(drive::stop)),
                goToPosition(Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_ALLIANCE_SIDE, Constants.RED_RIGHT_ALLIANCE_SIDE),
                goToPosition(Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET, Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shootingInPlace(), runOnce(drive::stop)));
    }

    public Command leftStarting_neutralZone_shoot_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                goToPosition(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE)
                        .raceWith(zeroIntakeAndHood().andThen(intake.deploy())),
                intaking(Constants.BLUE_LEFT_CENTER_LINE, Constants.RED_LEFT_CENTER_LINE, ()-> null, ()-> null),
                goToPosition(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shooting(5.0), runOnce(drive::stop)),
                goToPosition(Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_ALLIANCE_SIDE, Constants.RED_LEFT_ALLIANCE_SIDE),
                goToPosition(Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                Commands.parallel(shootingInPlace(), runOnce(drive::stop)));
    }

    public Command leftStarting_neutralZone_neutralZone_fullPasses() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZeroTowardsPosition(Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intakingWithDistanceThreshold(Constants.BLUE_MIDDLE_CENTER_LINE_ROTATED_RIGHT, Constants.RED_MIDDLE_CENTER_LINE_ROTATED_RIGHT,
                        getDistanceToPose(Constants.BLUE_MIDDLE_CENTER_LINE_ROTATED_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT)),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE_ROTATED, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE_ROTATED),
                shootingInPlace(),
                intakeAndPassTowardsPosition(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_LEFT,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_WITH_X_OFFSET, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE_WITH_X_OFFSET,
                        getDistanceToPose(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_LEFT)),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_ROTATED_LEFT, () -> Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET,
                        getDistanceToPose(Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_ROTATED_LEFT, Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET)),
                shootingInPlace()
        );
    }

    public Command rightStarting_neutralZone_neutralZone_fullPasses() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZeroTowardsPosition(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intakingWithDistanceThreshold(Constants.BLUE_MIDDLE_CENTER_LINE_ROTATED_LEFT, Constants.RED_MIDDLE_CENTER_LINE_ROTATED_LEFT,
                        getDistanceToPose(Constants.BLUE_MIDDLE_CENTER_LINE_ROTATED_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_FROM_LEFT)),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE_FROM_RIGHT, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE_ROTATED),
                Commands.parallel(shootingInPlace(), runOnce(drive::stop)),
                intakeAndPassTowardsPosition(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_ROTATED,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE_WITH_X_OFFSET, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE_WITH_X_OFFSET,
                        getDistanceToPose(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_LEFT, Constants.RED_MIDDLE_ALLIANCE_SIDE_ROTATED)),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET,
                        getDistanceToPose(Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET, Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE_WITH_Y_OFFSET)),
                shootingInPlace()
        );
    }

    public Command middleStarting_shootIntoHub() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_IN_FRONT_OF_HUB_AUTO_LINE, Constants.RED_IN_FRONT_OF_HUB_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPositionAndShoot(Constants.BLUE_HUB_SHOOTING, Constants.RED_HUB_SHOOTING),
                shootingInPlace(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        );
    }

    public Command leftStarting_shootIntoHub(){
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPositionAndShoot(Constants.BLUE_LEFT_UNDER_TRENCH_SHOOTING, Constants.RED_LEFT_UNDER_TRENCH_SHOOTING),
                shootingInPlace(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        );
    }

    public Command rightStarting_shootIntoHub() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroIntakeAndHood(),
                goToPositionAndShoot(Constants.BLUE_RIGHT_UNDER_TRENCH_SHOOTING, Constants.RED_RIGHT_UNDER_TRENCH_SHOOTING),
                shootingInPlace(Units.degreesToRadians(40.0)).alongWith(runOnce(drive::stop))
        );
    }

    public Command rightStarting_neutralZone_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakingAndZeroing(Constants.BLUE_RIGHT_CENTER_LINE, Constants.RED_RIGHT_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE,
                        Units.feetToMeters(5.0)),
                shootingInPlace(),
                intaking(Constants.BLUE_RIGHT_ALLIANCE_SIDE, Constants.RED_RIGHT_ALLIANCE_SIDE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE,
                        Units.feetToMeters(5.0)),
                shootingInPlace()
        );
    }

    public Command leftStarting_neutralZone_neutralZone() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intakingAndZeroing(Constants.BLUE_LEFT_CENTER_LINE, Constants.RED_LEFT_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE,
                        Units.feetToMeters(5.0)),
                shootingInPlace(),
                intaking(Constants.BLUE_LEFT_ALLIANCE_SIDE, Constants.RED_LEFT_ALLIANCE_SIDE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_INTERMEDIATE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                goToPosition(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE,
                        Units.feetToMeters(5.0)),
                shootingInPlace()
        );
    }


    public Command leftSide_depot_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_DEPOT_AUTO_LINE, Constants.RED_DEPOT_AUTO_LINE),
                goToPositionAndShoot(Constants.BLUE_LEFT_DEPOT, Constants.RED_LEFT_DEPOT,
                        () -> Constants.BLUE_STARTING_TO_DEPOT_INTERMEDIATE, () -> Constants.RED_STARTING_TO_DEPOT_INTERMEDIATE, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD)),
                intakeAndShoot(Constants.BLUE_RIGHT_DEPOT, Constants.RED_RIGHT_DEPOT),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB));
    }

    public Command rightSide_outpost_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_AUTO_LINE, Constants.RED_RIGHT_AUTO_LINE),
                goToPositionShootAndZero(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_OUTPOST_INTERMEDIATE, () -> Constants.RED_OUTPOST_INTERMEDIATE));
    }

    public Command leftStartingNoShooting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace());
    }

    public Command rightStartingNoShooting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace());
    }

    public Command leftStartingNoShooting_neutralZone_middleLine_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command rightStartingNoShooting_neutralZone_middleLine_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB,
                        () -> Constants.BLUE_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command leftStartingNoShooting_neutralZone_AllianceSide_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE)

        );
    }

    public Command rightStartingNoShooting_neutralZone_AllianceSide_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB,
                        () -> Constants.BLUE_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command rightStartingNoShooting_neutralZone_middleLine_x2() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                intakePassAndZero(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace()
        );
    }

    public Command leftStarting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace());
    }

    public Command rightStarting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace());
    }

    public Command leftStarting_neutralZone_middleLine_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command rightStarting_neutralZone_middleLine_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB,
                        () -> Constants.BLUE_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command leftStarting_neutralZone_AllianceSide_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_RIGHT_TRENCH_TO_CLIMB_INTERMEDIATE)

        );
    }

    public Command rightStarting_neutralZone_AllianceSide_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB,
                        () -> Constants.BLUE_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE, () -> Constants.RED_LEFT_TRENCH_TO_CLIMB_INTERMEDIATE));
    }

    public Command leftStarting_neutralZone_middleLine_x2() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace()
        );
    }

    public Command rightStarting_neutralZone_middleLine_x2() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, () -> Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace(),
                intakeAndPass(Constants.BLUE_MIDDLE_ALLIANCE_SIDE_FROM_RIGHT, Constants.RED_MIDDLE_ALLIANCE_SIDE,
                        () -> Constants.BLUE_LEFT_ALLIANCE_SIDE, () -> Constants.RED_LEFT_ALLIANCE_SIDE_INTERMEDIATE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE, () -> Constants.RED_RIGHT_ALLIANCE_SIDE_TO_TRENCH_INTERMEDIATE),
                shootingInPlace()
        );
    }

    public Command leftStarting_depot_outpost_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_DEPOT_AUTO_LINE, Constants.RED_DEPOT_AUTO_LINE),
                goToPositionAndShoot(Constants.BLUE_LEFT_DEPOT, Constants.RED_LEFT_DEPOT,
                        () -> Constants.BLUE_STARTING_TO_DEPOT_INTERMEDIATE, () -> Constants.RED_STARTING_TO_DEPOT_INTERMEDIATE, Units.feetToMeters(DEFAULT_ANGLE_DISTANCE_THRESHOLD)),
                intakeAndShoot(Constants.BLUE_RIGHT_DEPOT, Constants.RED_RIGHT_DEPOT),
                goToPositionAndShoot(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED,
                        () -> Constants.BLUE_AROUND_TOWER_TO_OUTPOST_INTERMEDIATE, () -> Constants.RED_AROUND_TOWER_TO_OUTPOST_INTERMEDIATE),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_OUTPOST_INTERMEDIATE, () -> Constants.RED_OUTPOST_INTERMEDIATE));
    }

    public Command rightStarting_outpost_depot_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_AUTO_LINE, Constants.RED_RIGHT_AUTO_LINE),
                goToPositionShootAndZero(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED),
                goToPositionAndShoot(Constants.BLUE_OUTPOST_INTERMEDIATE, Constants.RED_OUTPOST_INTERMEDIATE),
                goToPositionAndShoot(Constants.BLUE_LEFT_DEPOT, Constants.RED_LEFT_DEPOT,
                        () -> Constants.BLUE_AROUND_TOWER_TO_DEPOT_INTERMEDIATE, () -> Constants.RED_AROUND_TOWER_TO_DEPOT_INTERMEDIATE),
                intakeAndShoot(Constants.BLUE_RIGHT_DEPOT, Constants.RED_RIGHT_DEPOT),
                goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB));
    }

    public Command rightStarting_pickupNeutral_pickupNeutral_Outpost() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShootInPlace(),
                intaking(Constants.BLUE_RIGHT_CENTER_LINE_INTERMEDIATE, Constants.RED_RIGHT_CENTER_LINE_INTERMEDIATE),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                shootingInPlace(),
                intaking(Constants.BLUE_RIGHT_ALLIANCE_SIDE_INTERMEDIATE, Constants.RED_RIGHT_ALLIANCE_SIDE_INTERMEDIATE),
                goToPosition(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                goToPositionAndShoot(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED),
                shootingInPlace());
    }
}
