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

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

    public Command aim(DoubleSupplier xSupplier, DoubleSupplier ySupplier, boolean aimAtHub) {
        return parallel(shooter.follow(() -> firingSolution.hoodAngle(), () -> 0.0, () -> firingSolution.shooterVelocity()),
                drive.driveWithAngle(xSupplier, ySupplier, () -> firingSolution.turretAngle()),
                run(() -> firingSolution = getFiringSolution(aimAtHub))
        ).beforeStarting(runOnce(() -> firingSolution = getFiringSolution(aimAtHub))).finallyDo(() -> firingSolution = null);
    }

    private FiringSolution getFiringSolution(boolean aimAtHub) {
        return fireControlSystem.calculate(
                drive.getPose(), drive.getFieldOrientedVelocity(),
                getShooterTarget(drive.getPose(), isRedAlliance(), aimAtHub), aimAtHub);
    }

    public Command shooterDefault() {
        return shooter.follow(() -> 0.0, () -> 0.0, () -> Constants.Shooter.DEFAULT_VELOCITY);
    }

    public Command calibrateShooter(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier hoodAngleSupplier, DoubleSupplier shooterVelocitySupplier, boolean aimAtHub) {
        Supplier<FiringSolution> firingSolution = () -> getFiringSolution(aimAtHub);

        return shooter.follow(hoodAngleSupplier, () -> 0.0, shooterVelocitySupplier).alongWith(drive.driveWithAngle(xSupplier, ySupplier, () -> firingSolution.get().turretAngle()));
    }

    private Translation2d getShooterTarget(Pose2d robot, boolean redAlliance, boolean aimAtHub) {
        Translation2d target;
        if (aimAtHub)
            target = redAlliance ? Constants.Shooter.RED_HUB : Constants.Shooter.BLUE_HUB;
        else {
            Translation2d lower = redAlliance ? Constants.Shooter.PASS_DEPOT_RED : Constants.Shooter.PASS_OUTPOST_BLUE;
            Translation2d upper = redAlliance ? Constants.Shooter.PASS_OUTPOST_RED : Constants.Shooter.PASS_DEPOT_BLUE;

            target = robot.getY() > (upper.getY() + lower.getY()) / 2 ? upper : lower;
        }
        Logger.recordOutput("CommandFactory/ShooterTarget", new Pose2d(target, Rotation2d.kZero));
        return target;
    }

    private Pose2d getTurretPose() {
        return FireControlSystem.getTurretPose(drive.getPose());
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
                hopper.withVoltage(4.0, 4.0),
                feeder.withVoltage(4.0)
        );
    }
}