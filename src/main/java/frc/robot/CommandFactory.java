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

import java.util.function.BooleanSupplier;
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

        return shooter.follow(() -> fireControlSystem.calculate(getTurretPose(), drive.getVelocity(), target.get(), aimAtHub.getAsBoolean()));
    }

    public Translation2d getShooterTarget(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        if (aimAtHub)
            return redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
        else {
            Translation2d upper = redAlliance ? Constants.Shooter.UPPER_PASS_RED : Constants.Shooter.UPPER_PASS_BLUE;
            Translation2d lower = redAlliance ? Constants.Shooter.LOWER_PASS_RED : Constants.Shooter.LOWER_PASS_BLUE;

            return robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower;
        }
    }

    public Pose2d getTurretPose() {
        Rotation2d angle = new Rotation2d(drive.getRotation().getRadians() + shooter.getTurretPosition() + Constants.Shooter.TURRET_OFFSET_ANGLE);
        Translation2d turretTranslation = drive.getPose().getTranslation().plus(new Translation2d(Constants.Shooter.TURRET_OFFSET_DISTANCE, angle));
        return new Pose2d(turretTranslation, angle);
    }
}