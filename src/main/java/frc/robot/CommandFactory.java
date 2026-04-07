package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FireControlSystem;
import frc.robot.util.FiringSolution;
import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.*;

public class CommandFactory {
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;

    private final FireControlSystem fireControlSystem = new FireControlSystem();

    private FiringSolution firingSolution = null;

    public CommandFactory(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
    }

    public boolean isAimingAtTarget() {
        if (firingSolution == null) {
            Logger.recordOutput("CommandFactory/ShooterIsAimingAtTarget", false);
            return false;
        }

        boolean atTarget = this.shooter.isAt(firingSolution.hoodAngle(), firingSolution.turretAngle(), firingSolution.shooterVelocity());
        Logger.recordOutput("CommandFactory/ShooterIsAimingAtTarget", atTarget);
        return atTarget;
    }

    public Command aim(BooleanSupplier aimAtHub) {
        return shooter.follow(() -> {
                    Pose2d robotPose = drive.getPose();

                    Translation2d targetPosition = getShooterTarget(robotPose, isRedAlliance(), aimAtHub.getAsBoolean());

                    firingSolution = fireControlSystem.calculate(
                            drive.getPose(), drive.getFieldOrientedVelocity(),
                            Rotation2d.fromRadians(shooter.getTurretPosition()),
                            targetPosition, aimAtHub.getAsBoolean());
                    return firingSolution;
                })
                .finallyDo(() -> firingSolution = null);
    }

    public Command aim(boolean aimAtHub) {
        return aim(()-> aimAtHub);
    }

    public Command aimAndZeroHood(boolean aimAtHub) {
        return shooter.follow(() -> {
                    Pose2d robotPose = drive.getPose();

                    Translation2d targetPosition = getShooterTarget(robotPose, isRedAlliance(), aimAtHub);

                    firingSolution = fireControlSystem.calculate(
                            drive.getPose(), drive.getFieldOrientedVelocity(),
                            Rotation2d.fromRadians(shooter.getTurretPosition()),
                            targetPosition, aimAtHub);
                    return firingSolution;
                })
                .beforeStarting(shooter::zeroHood)
                .finallyDo(() -> firingSolution = null);
    }

    public Command shooterDefault(BooleanSupplier shootingIntoHub) {
        return shooter.follow(() -> 0.0, () -> {
            Logger.recordOutput("CommandFactory/shootingIntoHub", shootingIntoHub.getAsBoolean());
            Translation2d target = getShooterTarget(drive.getPose(), isRedAlliance(), shootingIntoHub.getAsBoolean());
            return fireControlSystem.calculate(
                            drive.getPose(), drive.getFieldOrientedVelocity(),
                            Rotation2d.fromRadians(shooter.getTurretPosition()), target, shootingIntoHub.getAsBoolean())
                    .turretAngle();
        }, () -> Constants.Shooter.DEFAULT_VELOCITY);
    }

    public Command calibrateShooter(DoubleSupplier hoodAngleSupplier, DoubleSupplier shooterVelocitySupplier, BooleanSupplier aimAtHub) {
        return shooter.follow(() -> {
            Pose2d robotPose = drive.getPose();

            Translation2d targetPosition = getShooterTarget(robotPose, isRedAlliance(), aimAtHub.getAsBoolean());

            var firingSolution = fireControlSystem.calculate(
                    drive.getPose(), drive.getFieldOrientedVelocity(),
                    Rotation2d.fromRadians(shooter.getTurretPosition()),
                    targetPosition, aimAtHub.getAsBoolean());
            return new FiringSolution(firingSolution.turretAngle(), hoodAngleSupplier.getAsDouble(),
                    shooterVelocitySupplier.getAsDouble());
        });
    }

    public static Translation2d getShooterTarget(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        Translation2d target;
        if (aimAtHub)
            target = redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
        else {
            Translation2d upper = redAlliance ? Constants.Shooter.PASS_OUTPOST_RED : Constants.Shooter.PASS_DEPOT_BLUE;
            Translation2d lower = redAlliance ? Constants.Shooter.PASS_DEPOT_RED : Constants.Shooter.PASS_OUTPOST_BLUE;

            target = robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower;
        }
        Logger.recordOutput("CommandFactory/ShooterTarget", new Pose2d(target, Rotation2d.kZero));
        return target;
    }

    private Pose2d getTurretPose() {
        return FireControlSystem.getTurretPose(drive.getPose(), Rotation2d.fromRadians(shooter.getTurretPosition()));
    }

    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Feed fuel into the shooter from the hopper
     * <p>
     * Requires: Feeder, Hopper
     */
    public Command feedIntoShooter() {
        return parallel(
                hopper.withVoltage(6.0, 6.0),
                feeder.withVoltage(6.0)
        );
    }

    public boolean isHubShootingMode() {
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue) {
            return (drive.getPose().getX() < Constants.BLUE_ALLIANCE_LINE_X);
        } else {
            return (drive.getPose().getX() > Constants.RED_ALLIANCE_LINE_X);
        }
    }

    public boolean shouldShoot() {
        var notPassingBehindHub = true;
        var notShootingUnderTower = true;
        var poseX = getTurretPose().getX();
        var poseY = getTurretPose().getY();

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue) {
            if (poseX < Constants.BEHIND_HUB_X && poseX > Constants.BLUE_ALLIANCE_LINE_X &&
                    poseY < Constants.BEHIND_HUB_LARGER_Y && poseY > Constants.BEHIND_HUB_SMALLER_Y) {
                notPassingBehindHub = false;
            }
        } else {
            if (poseX > Constants.BEHIND_HUB_X && poseX < Constants.RED_ALLIANCE_LINE_X &&
                    poseY < Constants.BEHIND_HUB_LARGER_Y && poseY > Constants.BEHIND_HUB_SMALLER_Y) {
                notPassingBehindHub = false;
            }
        }

        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                DriverStation.Alliance.Blue) {
            if (poseX < Constants.BLUE_UNDER_TOWER_X && poseX > 0.0 &&
                    poseY < Constants.UNDER_TOWER_LARGER_Y && poseY > Constants.UNDER_TOWER_SMALLER_Y) {
                notShootingUnderTower = false;
            }
        } else {
            if (poseX > Constants.RED_UNDER_TOWER_X && poseX < Constants.FIELD.getFieldLength() &&
                    poseY < Constants.UNDER_TOWER_LARGER_Y && poseY > Constants.UNDER_TOWER_SMALLER_Y) {
                notShootingUnderTower = false;
            }
        }

        var linearVelocity = Math.sqrt(Math.pow(drive.getFieldOrientedVelocity().vxMetersPerSecond, 2.0)
                + Math.pow(drive.getFieldOrientedVelocity().vyMetersPerSecond, 2.0));
        var rotationalVelocity = drive.getFieldOrientedVelocity().omegaRadiansPerSecond;
        var velocityBelowShootingMax = (!(linearVelocity > drive.getMaxLinearSpeedMetersPerSec() * 0.4))
                || (!(rotationalVelocity > drive.getMaxAngularSpeedRadPerSec() * 0.5));

        Logger.recordOutput("CommandFactory/ShooterAtTargets", shooter.atTargets());
        Logger.recordOutput("CommandFactory/NotPassingBehindHub", notPassingBehindHub);
        Logger.recordOutput("CommandFactory/VelocityBelowShootingMax", velocityBelowShootingMax);

        return (shooter.atTargets() && notPassingBehindHub && velocityBelowShootingMax && notShootingUnderTower);
    }

    public Command autonomousFeedAndShoot(boolean aimAtHub, double pivotAngle) {
        return aim(aimAtHub)
                .alongWith(
                        repeatingSequence(
                                waitSeconds(0.1),
                                waitUntil(shooter::atShooterTargetVelocity),
                                feedIntoShooter().onlyWhile(shooter::atShooterTargetVelocity)
                        ), intake.feedWithAngle(pivotAngle));
    }

    public Command autonomousFeedShootAndZero(boolean aimAtHub) {
        return aimAndZeroHood(aimAtHub)
                .alongWith(feedIntoShooter()
                        .onlyWhile(shooter::atShooterTargetVelocity))
                .alongWith(intake.feed()
                        .beforeStarting(intake::zeroPivot));
    }

    public Command autonomousFeedAndShootWithoutIntake(boolean aimAtHub, boolean shouldZero) {
        return aim(aimAtHub)
                .alongWith((feedIntoShooter().onlyWhile(shooter::atShooterTargetVelocity))
                        .beforeStarting(shouldZero ? shooter.zeroHood() : Commands.none()));
    }
}