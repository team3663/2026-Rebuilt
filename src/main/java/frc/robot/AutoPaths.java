package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class AutoPaths {
    private final Drive drive;
    private final Intake intake;
    private final Shooter shooter;

    private Pose2d goToPositionTarget;

    public AutoPaths(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.intake = intake;
        this.shooter = shooter;
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
     * @see #goToPosition(Pose2d)
     */
    private Command goToPosition(Supplier<Pose2d> targetPose, Supplier<Pose2d> intermediatePoseSupplier, BooleanSupplier slowAccel) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};

        return drive.goToPosition(() -> {
                            Pose2d target;
                            if (intermediateHolder[0] != null && ((intermediateHolder[0] = intermediatePoseSupplier.get()) != null))
                                target = intermediateHolder[0];
                            else
                                target = targetPose.get();
                            goToPositionTarget = target;
                            return target;
                        },
                        () -> slowAccel.getAsBoolean(),
                        () -> drive.getMaxLinearSpeedMetersPerSec())
                .beforeStarting(() -> {
                    if (intermediatePoseSupplier != null) intermediateHolder[0] = intermediatePoseSupplier.get();
                    else Commands.none();
                });
    }

    /**
     * Drives the robot to the target position
     * <p>
     * this command requires {@link Drive} and does not end
     * </p>
     *
     * @param targetPose the target position of the robot
     * @see #goToPosition(Pose2d, Pose2d)
     * @see #goToPosition(Supplier, Supplier, BooleanSupplier)
     */
    private Command goToPosition(Pose2d targetPose) {
        return goToPosition(() -> targetPose, () -> null, () -> false);
    }

    /**
     * Drives the robot to the target position
     * <p>
     * This command requires {@link Drive} and does not end
     * </p>
     *
     * @param blueTargetPose target position of the robot if on blue alliance
     * @param redTargetPose  target position of the robot if on red alliance
     * @see #goToPosition(Pose2d)
     */
    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPosition(alliancePose(blueTargetPose, redTargetPose));
    }

    private Command goToPosition(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueAllianceIntermediateSupplier, Supplier<Pose2d> redAllianceIntermediateSupplier) {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};

        return goToPosition(
                () -> alliancePose(blueTargetPose, redTargetPose),
                () -> {
                    if (intermediatePoseHolder[0] != null) {
                        if (!drive.atPosition(intermediatePoseHolder[0].getTranslation(), Units.feetToMeters(3.5)))
                            return intermediatePoseHolder[0];
                        else return null;
                    } else return null;
                },
                () -> false)
                .beforeStarting(() -> {
                    if (blueAllianceIntermediateSupplier != null)
                        intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediateSupplier.get(), redAllianceIntermediateSupplier.get());
                    else Commands.none();
                });
    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Drive} and does not end
     *
     * @param blueTargetPose               target position of the robot if on blue alliance
     * @param redTargetPose                target position of the robot if on red alliance
     * @param blueIntermediatePoseSupplier a supplier that is possibly {@code null} of intermediate positions to travel to if on blue alliance
     * @param redIntermediatePoseSupplier  a supplier that is possibly {@code null} of intermediate positions to travel to if on red alliance
     * @see #intaking(Pose2d, Pose2d) for an option with no intermediate suppliers
     */
    private Command intaking(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePoseSupplier, Supplier<Pose2d> redIntermediatePoseSupplier) {
        return goToPosition(blueTargetPose, redTargetPose,
                blueIntermediatePoseSupplier, redIntermediatePoseSupplier)
                .withDeadline(Commands.sequence(
                                intake.deployAndIntake())
                        .andThen(intake.stow()))
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Intake}
     *
     * @param blueTargetPose target position of the robot if on blue alliance
     * @param redTargetPose  target position of the robot if on red alliance
     * @see #intaking(Pose2d, Pose2d, Supplier, Supplier)
     */
    private Command intaking(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intaking(blueTargetPose, redTargetPose, () -> null, () -> null);
    }

    /**
     * Runs the shooter and has an option to zero subsystems
     * <p>
     * This command requires {@link Shooter}
     *
     * @see #shooting()
     * @see #zeroAndShoot() a zeroing alternative
     */
    // TODO wait till shooting command gets merged into main
    // TODO add zeroing for pivot and climber if necessary
    private Command shooting() {
        return Commands.none();
    }

    /**
     * Runs the shooter and zeros the Intake and Climber subsystems
     * <p>
     * This command requires {@link Shooter} {@link Intake}
     */
    private Command zeroAndShoot() {
        return shooting().alongWith(intake.zeroPivot());
    }

    private Command goToPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose, boolean shouldZero) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(shouldZero ? zeroAndShoot() : shooting())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPositionAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose, boolean shouldZero) {
        return goToPositionAndShoot(blueTargetPose, redTargetPose, () -> null, () -> null, shouldZero);
    }

    private Command intakeAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose,
                                   Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose,
                                   boolean shouldZero) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(shouldZero ? zeroAndShoot() : Commands.none())
                .alongWith(shooting())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command intakeAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose, boolean shouldZero) {
        return intakeAndShoot(blueTargetPose, redTargetPose, () -> null, () -> null, shouldZero);
    }

    private Command intakeAndShoot(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return intakeAndShoot(blueTargetPose, redTargetPose, () -> null, () -> null, false);
    }

    private Command intakeAndPass(Pose2d blueTargetPose, Pose2d redTargetPose, Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose) {
        return intaking(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                // TODO add passing command
                .alongWith(Commands.none())
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    private Command goToPositionAndClimb(Pose2d blueTargetPose, Pose2d redTargetPose,
                                         Supplier<Pose2d> blueIntermediatePose, Supplier<Pose2d> redIntermediatePose) {
        return goToPosition(blueTargetPose, redTargetPose, blueIntermediatePose, redIntermediatePose)
                .alongWith(shooting());
    }

    private Command goToPositionAndClimb(Pose2d blueTargetPose, Pose2d redTargetPose) {
        return goToPositionAndClimb(blueTargetPose, redTargetPose, () -> null, () -> null)
                .until(() -> drive.atPosition(alliancePose(blueTargetPose, redTargetPose).getTranslation()));
    }

    // Auto Routines
    public Command leftSide_depot_leftClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_AUTO_LINE, Constants.RED_LEFT_AUTO_LINE),
                goToPositionAndShoot(Constants.BLUE_LEFT_DEPOT, Constants.RED_LEFT_DEPOT, true),
                intakeAndShoot(Constants.BLUE_RIGHT_DEPOT, Constants.RED_RIGHT_DEPOT),
                intake.stow().alongWith(goToPositionAndClimb(Constants.BLUE_LEFT_RUNG_CLIMB, Constants.RED_LEFT_RUNG_CLIMB)));
    }

    public Command rightSide_outpost_rightClimb() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_AUTO_LINE, Constants.RED_RIGHT_AUTO_LINE),
                goToPositionAndShoot(Constants.BLUE_OUTPOST_CENTERED, Constants.RED_OUTPOST_CENTERED, true),
                goToPositionAndClimb(Constants.BLUE_RIGHT_RUNG_CLIMB, Constants.RED_RIGHT_RUNG_CLIMB,
                        () -> Constants.BLUE_OUTPOST_INTERMEDIATE, () -> Constants.RED_OUTPOST_INTERMEDIATE));
    }

    public Command leftStarting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShoot().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE, () -> Constants.RED_LEFT_CENTER_LINE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH),
                shooting().alongWith(intake.stow()));
    }

    public Command rightStarting_neutralZone_middleLine() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShoot().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE, () -> Constants.RED_RIGHT_CENTER_LINE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH),
                shooting().alongWith(intake.stow()));
    }

    public Command leftStarting_neutralZone_middleLine_x2() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE,
                        Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShoot().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE, () -> Constants.RED_LEFT_CENTER_LINE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH),
//                shooting().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE, () -> Constants.RED_RIGHT_CENTER_LINE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH),
                shooting()
        );
    }

    public Command rightStarting_neutralZone_middleLine_x2() {
        return Commands.sequence(
                resetOdometry(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE),
                zeroAndShoot().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE, () -> Constants.RED_RIGHT_CENTER_LINE),
                intaking(Constants.BLUE_LEFT_UNDER_TRENCH_AUTO_LINE, Constants.RED_LEFT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_LEFT_CENTER_LINE_TO_TRENCH),
//                shooting().withTimeout(2.0),
                intakeAndPass(Constants.BLUE_MIDDLE_CENTER_LINE, Constants.RED_MIDDLE_CENTER_LINE,
                        () -> Constants.BLUE_LEFT_CENTER_LINE, () -> Constants.RED_LEFT_CENTER_LINE),
                intaking(Constants.BLUE_RIGHT_UNDER_TRENCH_AUTO_LINE, Constants.RED_RIGHT_UNDER_TRENCH_AUTO_LINE,
                        () -> Constants.BLUE_RIGHT_CENTER_LINE_TO_TRENCH, () -> Constants.RED_RIGHT_CENTER_LINE_TO_TRENCH),
                shooting()
        );
    }

}
