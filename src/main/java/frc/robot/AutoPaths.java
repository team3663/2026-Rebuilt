package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
        return Commands.defer(() -> drive.resetOdometry(() -> alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));
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
                        () -> false,
                        () -> drive.getMaxLinearSpeedMetersPerSec())
                .beforeStarting(() -> intermediateHolder[0] = intermediatePoseSupplier.get());
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
     * @param blueAlliancePose target position of the robot if on blue alliance
     * @param redAlliancePose  target position of the robot if on red alliance
     * @see #goToPosition(Pose2d)
     */
    private Command goToPosition(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
        return Commands.defer(() -> goToPosition(alliancePose(blueAlliancePose, redAlliancePose)), Set.of(drive));

    }

    private Command goToPosition(Pose2d blueAlliancePose, Pose2d redAlliancePose, Supplier<Pose2d> blueAllianceIntermediateSupplier, Supplier<Pose2d> redAllianceIntermediateSupplier) {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(
                        () -> alliancePose(blueAlliancePose, redAlliancePose),
                        () -> drive.atPosition(intermediatePoseHolder[0].getTranslation(),
                                Units.feetToMeters(1.0)) ? null : intermediatePoseHolder[0],
                        () -> false), Set.of(drive))
                .beforeStarting(() -> intermediatePoseHolder[0] = alliancePose(blueAllianceIntermediateSupplier.get(), redAllianceIntermediateSupplier.get()));
    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Drive} and does not end
     *
//     * @param targetPose               target position of the robot
//     * @param intermediatePoseSupplier a supplier that is possibly {@code null} of intermediate positions to travel to
//     * @see #intaking(Pose2d, Pose2d)
     */
    private Command intaking(Pose2d blueAlliancePose, Pose2d redAlliancePose, Supplier<Pose2d> blueIntermediatePoseSupplier, Supplier<Pose2d> redIntermediatePoseSupplier) {
        Pose2d[] targetPoseHolder = new Pose2d[]{null};

        return Commands.defer(() -> goToPosition(blueAlliancePose, redAlliancePose,
                        blueIntermediatePoseSupplier, redIntermediatePoseSupplier), Set.of(drive))
                .withDeadline(Commands.sequence(
                        intake.deployAndIntake()))
                .andThen(intake.stow())
                .beforeStarting(() -> targetPoseHolder[0] = alliancePose(blueAlliancePose, redAlliancePose));
    }

//    private Command intaking(Pose2d blueAlliancePose, Pose2d redAlliancePose, Supplier<Pose2d> blueIntermediatePoseSupplier, Supplier<Pose2d> redIntermediatePoseSupplier) {
//        return intaking(alliancePose(blueAlliancePose, redAlliancePose),
//                () -> alliancePose(blueIntermediatePoseSupplier.get(), redIntermediatePoseSupplier.get()));
//    }

    /**
     * Drives the robot to the target position while intaking
     * <p>
     * This command requires {@link Intake}
     *
     * @param blueAlliancePose target position of the robot if on blue alliance
     * @param redAlliancePose  target position of the robot if on red alliance
//     * @see #intaking(Pose2d, Supplier)
     * @see #intaking(Pose2d, Pose2d, Supplier, Supplier)
     */
//    private Command intaking(Pose2d blueAlliancePose, Pose2d redAlliancePose) {
//        return intaking(alliancePose(blueAlliancePose, redAlliancePose), ()-> null, ()-> null);
//    }

    /**
     * Runs the shooter and has an option to zero subsystems
     * <p>
     * This command requires {@link Shooter}
     *
     * @param shouldZero if we want to zero the intake pivot and climber
     * @see #shooting()
     * @see #zeroAndShoot() a zeroing alternative
     */
    // TODO wait till shooting command gets merged into main
    // TODO add zeroing for pivot and climber if necessary
    private Command shooting(boolean shouldZero) {
        return null;
    }

    /**
     * Runs the shooter
     * <p>
     * This command requires {@link Shooter}
     *
     * @see #zeroAndShoot() a zeroing alternative
     */
    private Command shooting() {
        return Commands.defer(() -> shooting(false), Set.of(shooter));
    }

    /**
     * Runs the shooter and zeros the Intake and Climber subsystems
     * <p>
     * This command requires {@link Shooter} {@link Intake}
     */
    private Command zeroAndShoot() {
        return Commands.defer(() -> shooting(true), Set.of(shooter));
    }

    // TODO fix this so that we don't start intaking until after zeroed
    private Command intakeAndShoot(Pose2d blueAlliancePose, Pose2d redAlliancePose,
                                   Supplier<Pose2d> intermediatePoseSupplier, boolean shouldZero) {
        return null;
//        return Commands.defer(() -> shooting(shouldZero), Set.of(shooter))
//                .alongWith(intaking(blueAlliancePose, redAlliancePose, intermediatePoseSupplier));
    }

    public Command testAuto() {
        Pose2d[] intermediatePoseHolder = new Pose2d[]{null};
        return Commands.sequence(
                resetOdometry(new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(Constants.FIELD.getFieldLength(), Constants.FIELD.getFieldWidth(),
                                new Rotation2d(Units.degreesToRadians(180)))).alongWith(intake.zeroPivot()),
                intaking(new Pose2d(Units.feetToMeters(10), Units.inchesToMeters(5), new Rotation2d(Units.degreesToRadians(180))),
                        new Pose2d(Constants.FIELD.getFieldLength() - Units.feetToMeters(10), Constants.FIELD.getFieldWidth() - Units.feetToMeters(5), new Rotation2d(0)),
                        () -> new Pose2d(Units.feetToMeters(5), Units.feetToMeters(2.5), new Rotation2d(Units.degreesToRadians(270))),
                        () -> new Pose2d(Constants.FIELD.getFieldLength() - Units.feetToMeters(5), Constants.FIELD.getFieldWidth() - Units.feetToMeters(2.5),
                                new Rotation2d(Units.degreesToRadians(270)))));
    }


}
