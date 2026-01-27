package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.Set;
import java.util.function.Supplier;

public class AutoPaths {
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;

    private Pose2d goToPositionTarget;

    public AutoPaths(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
    }

    /**
     * Returns the pose of the correct alliance
     *
     * @param blueAlliancePose the pose to return when the robot is on the blue alliance
     * @param redAlliancePose  the pose to return when the robot is on the red alliance
     */
    private Pose2d alliancePose(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue ? blueAlliancePose : redAlliancePose;
    }

    /**
     * Resetting the odometry of the drivetrain based on the robot's alliance
     * <p>
     * requires the {@link Drive} subsystem and executes instantly
     * </p>
     *
     * @param blueAlliancePose the pose to reset the odometry to if on the blue alliance
     * @param redAlliancePose  the pose to reset the odometry to if on the red alliance
     */
    private Command resetOdometry(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> drive.resetOdometry(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));
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
     * @see #goToPosition(Pose2d, Pose2d)
     * @see #goToPosition(Pose2d) 
     * @param targetPose               the target position of the robot
     * @param intermediatePoseSupplier a supplier that is possibly {@code null} of intermediate positions to travel to
     */
    private Command goToPosition(Pose2d targetPose, Supplier<Pose2d> intermediatePoseSupplier, boolean shouldZero, boolean intake, boolean shoot) {
        Pose2d[] intermediateHolder = new Pose2d[]{null};

        return drive.goToPosition(
                drive,
                () -> {
                    Pose2d target;
                    if (intermediateHolder[0] != null && (intermediateHolder[0] = intermediatePoseSupplier.get()) != null) {
                        target = intermediateHolder[0];
                    } else {
                        target = targetPose;
                    }
                    goToPositionTarget = target;
                    return target;
                },
                () -> false,
                () -> drive.getMaxLinearSpeedMetersPerSec()
        ).beforeStarting(() -> intermediateHolder[0] = intermediatePoseSupplier.get());
    }

    /**
     * Drives the robot to the target position
     * <p>
     * this command requires {@link Drive} and does not end
     * </p>
     * @see #goToPosition(Pose2d, Pose2d)
     * @see #goToPosition(Pose2d, Supplier, boolean, boolean, boolean)
     * @param targetPose the target position of the robot
     */
    private Command goToPosition(Pose2d targetPose) {
        return goToPosition(targetPose, () -> null, false, false, false);
    }

    /**
     * Drives the robot to the target position
     * <p>
     * This command requires {@link Drive} and does not end
     * </p>
     * @see #goToPositionWhileShooting(Pose2d)
     * @see #goToPosition(Pose2d, Supplier, boolean, boolean, boolean)
     * @param blueAlliancePose target position of the robot if on blue alliance
     * @param redAlliancePose  target position of the robot if on red alliance
     */
    private Command goToPosition(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> goToPosition(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));

    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Drive} and does not end
     * @see #goToPositionWhileIntaking(Pose2d,Pose2d)
     * @param targetPose target position of the robot
     */
    private Command goToPositionWhileIntaking(Pose2d targetPose) {
        return goToPosition(targetPose, () -> null, false, true, false);
    }

    /**
     * Drives the robot to the target position while intaking
     * @see #goToPositionWhileIntaking(Pose2d)
     * @param blueAlliancePose target position of the robot if on blue alliance
     * @param redAlliancePose target position of the robot if on red alliance
     */
    private Command goToPositionWhileIntaking(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> goToPositionWhileIntaking(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));
    }

    /**
     * Drives the robot to the target position while shooting
     * @param targetPose target position of the robot
     * @see #goToPositionWhileShooting(Pose2d, Pose2d)
     */
    private Command goToPositionWhileShooting(Pose2d targetPose) {
        return goToPosition(targetPose, () -> null, false, false, true);
    }

    /**
     *  Drives the robot to the target position while shooting
     * @see #goToPositionWhileShooting(Pose2d)
     * @param blueAlliancePose target position of the robot if on blue alliance
     * @param redAlliancePose target position of the robot if on red alliance
     */
    private Command goToPositionWhileShooting(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> goToPositionWhileShooting(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));
    }


}
