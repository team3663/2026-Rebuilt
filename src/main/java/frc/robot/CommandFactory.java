package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import java.util.function.BooleanSupplier;

public class CommandFactory {
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;

    public CommandFactory(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
    }

    public Command aimShooter(BooleanSupplier aimAtHub) {
        var alliance = DriverStation.getAlliance();
        BooleanSupplier redAlliance = () -> alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        return shooter.followWithShooter(
                () -> getTurretAngle(drive.getPose(), redAlliance.getAsBoolean(), aimAtHub.getAsBoolean()),
                () -> 0.0);
//                () -> getHoodAngle(drive.getPose(), redAlliance.getAsBoolean(), aimAtHub.getAsBoolean()));
    }

    public double getTurretAngle(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        double turretAngle;
        if (aimAtHub) {
            turretAngle = getAngle(robot, redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB);
        } else {
            Pose2d upper = redAlliance ? Constants.Shooter.UPPER_PASS_RED : Constants.Shooter.UPPER_PASS_BLUE;
            Pose2d lower = redAlliance ? Constants.Shooter.LOWER_PASS_RED : Constants.Shooter.LOWER_PASS_BLUE;

            turretAngle = getAngle(robot, robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower);
        }
        return turretAngle;
    }

    public double getAngle(Pose2d current, Pose2d target) {
        return getAngle(current, target, current.getRotation().getRadians());
    }

    public double getAngle(Pose2d current, Pose2d target, double angleOffset) {
        double absoluteAngle = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX());
        return absoluteAngle - angleOffset;
    }
}