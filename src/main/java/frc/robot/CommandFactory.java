package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FireControlSystem;
import frc.robot.util.FiringSolution;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

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
            return false;
        }

        return this.shooter.isAt(firingSolution.hoodAngle(), firingSolution.turretAngle(), firingSolution.shooterVelocity());
    }

    public Command aim(boolean aimAtHub) {
        return shooter.follow(() -> {
                    Pose2d robotPose = drive.getPose();

                    Translation2d targetPosition = getShooterTarget(robotPose, isRedAlliance(), aimAtHub);

                    firingSolution = fireControlSystem.calculate(
                            drive.getPose(), drive.getFieldOrientedVelocity(),
                            Rotation2d.fromRadians(shooter.getTurretPosition()),
                            targetPosition, aimAtHub);
                    return firingSolution;
                })
                .finallyDo(() -> firingSolution = null);
    }

    public Command shooterDefault() {
        return shooter.follow(() -> 0.0, () -> {
            Translation2d target = isRedAlliance() ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
            return target.minus(drive.getPose().getTranslation()).getAngle().getRadians();
        }, () -> Constants.Shooter.DEFAULT_VELOCITY);
    }

    private Translation2d getShooterTarget(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        Translation2d target;
        if (aimAtHub)
            target = redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
        else {
            Translation2d upper = redAlliance ? Constants.Shooter.UPPER_PASS_RED : Constants.Shooter.UPPER_PASS_BLUE;
            Translation2d lower = redAlliance ? Constants.Shooter.LOWER_PASS_RED : Constants.Shooter.LOWER_PASS_BLUE;

            target = robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower;
        }
        Logger.recordOutput("CommandFactory/ShooterTarget", new Pose2d(target, Rotation2d.kZero));
        return target;
    }

    private boolean isRedAlliance() {
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
                hopper.withVoltage(4.0),
                feeder.withVoltage(4.0)
        );
    }
}