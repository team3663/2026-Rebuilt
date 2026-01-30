package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FireControlSystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class CommandFactory {
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;

    private final FireControlSystem fireControlSystem = new FireControlSystem();

    public CommandFactory(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter) {
        this.drive = drive;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
    }

    public Command aimShooter(BooleanSupplier aimAtHub) {
        var alliance = DriverStation.getAlliance();
        boolean redAlliance = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

        Supplier<Translation2d> target = () -> getShooterTarget(drive.getPose(), redAlliance, aimAtHub.getAsBoolean());
        DoubleSupplier targetHeight = () -> aimAtHub.getAsBoolean() ? Constants.Shooter.HUB_HEIGHT : Constants.Shooter.PASS_HEIGHT;

        return shooter.follow(() -> fireControlSystem.calculate(drive.getPose(), drive.getVelocity(), target.get(), targetHeight.getAsDouble()));
    }

    public Translation2d getShooterTarget(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        if (aimAtHub) {
            return redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
        } else {
            Translation2d upper = redAlliance ? Constants.Shooter.UPPER_PASS_RED : Constants.Shooter.UPPER_PASS_BLUE;
            Translation2d lower = redAlliance ? Constants.Shooter.LOWER_PASS_RED : Constants.Shooter.LOWER_PASS_BLUE;

            return robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower;
        }
    }

//    public double getAngle(Pose2d current, Pose2d target) {
//        return getAngle(current, target, current.getRotation().getRadians());
//    }
//
//    public double getAngle(Pose2d current, Pose2d target, double angleOffset) {
//        double absoluteAngle = Math.atan2(target.getY() - current.getY(), target.getX() - current.getX());
//        return absoluteAngle - angleOffset;
//    }
//
//    public double getHoodAngle(Pose2d robot, Pose2d target, double turretAngle, boolean aimHigh) {
//        double robotD = robot.getTranslation().getDistance(target.getTranslation());
//        double turretOffset = Constants.Shooter.TURRET_OFFSET_DISTANCE;
//        double turretToTargetAngle = turretAngle + Constants.Shooter.TURRET_OFFSET_ANGLE;
//        double distance = Math.sqrt(turretOffset * turretOffset + robotD * robotD -
//                2 * turretOffset * robotD * Math.cos(turretToTargetAngle));
//        double height = aimHigh ? Constants.Shooter.HUB_HEIGHT : Constants.Shooter.PASS_HEIGHT;
//        return getLaunchAngle(9.81, distance, height, shooter.getShootingOutputVelocity(),
//                Constants.Shooter.LAUNCH_HEIGHT, true);
//    }
//
//    // Finds the angle the ball needs to be launched at in order to go the target distance with taking in account:
//    //      gravity, initial and final heights, velocity, and if it should arc high or go straight to it
//    public double getLaunchAngle(double gravity, double distance, double targetHeight,
//                                 double launchVelocity, double launchHeight, boolean highShot) {
//        double dSquared = distance * distance;
//        double relativeHeight = targetHeight - launchHeight;
//        double velocitySquared = launchVelocity * launchVelocity;
//        double a = dSquared + relativeHeight * relativeHeight;
//        double b = (gravity * relativeHeight * dSquared) / (velocitySquared) - dSquared;
//        double c = gravity * gravity * dSquared * dSquared / (velocitySquared * velocitySquared);
//        double bSquaredMinusAC = b * b - a * c;
//        double solvedQuadratic = -b + (highShot ? bSquaredMinusAC : -bSquaredMinusAC) / (2 * a);
//        return Math.acos(Math.sqrt(solvedQuadratic));
//    }
}