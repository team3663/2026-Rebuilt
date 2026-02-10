package frc.robot.config;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.generated.C2026TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.feeder.C2026FeederIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.C2026HopperIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.C2026IntakeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.C2026ShooterIO;
import frc.robot.subsystems.shooter.Shooter;

public class C2026RobotFactory implements RobotFactory {
    @Override
    public Drive createDrive() {
        double odometryFrequencyHz = C2026TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
        PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(odometryFrequencyHz, C2026TunerConstants.kCANBus);

        Drive drive = new Drive(
                new GyroIOPigeon2(odometryThread, odometryFrequencyHz, C2026TunerConstants.DrivetrainConstants.Pigeon2Id,
                        C2026TunerConstants.DrivetrainConstants.Pigeon2Configs, C2026TunerConstants.kCANBus),
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2026TunerConstants.FrontLeft, C2026TunerConstants.kCANBus),
                C2026TunerConstants.FrontLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2026TunerConstants.FrontRight, C2026TunerConstants.kCANBus),
                C2026TunerConstants.FrontRight,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2026TunerConstants.BackLeft, C2026TunerConstants.kCANBus),
                C2026TunerConstants.BackLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2026TunerConstants.BackRight, C2026TunerConstants.kCANBus),
                C2026TunerConstants.BackRight);

        // Start odometry thread
        odometryThread.start();

        return drive;
    }

    @Override
    public Feeder createFeeder() {
        return new Feeder(new C2026FeederIO(
                new TalonFX(14),
                new CANrange(1)
        ));
    }

    @Override
    public Hopper createHopper() {
        return new Hopper(new C2026HopperIO(new TalonFX(10), new TalonFX(20)));
    }

    @Override
    public Intake createIntake() {
        return new Intake(new C2026IntakeIO(
                new TalonFX(11),
                new TalonFX(12),
                new TalonFX(13)
        ));
    }

    @Override
    public Shooter createShooter() {
        return new Shooter(new C2026ShooterIO(
                new TalonFX(16),
                new TalonFX(17),
                new TalonFX(18),
                new TalonFX(19),
                new CANcoder(7)
        ));
    }
}
