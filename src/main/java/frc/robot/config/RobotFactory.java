package frc.robot.config;

import frc.robot.generated.TestBotTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;

public interface RobotFactory {
    default Drive createDrive() {
        return new Drive(
                new GyroIO() {
                },
                new ModuleIO() {
                },
                TestBotTunerConstants.FrontLeft,
                new ModuleIO() {
                },
                TestBotTunerConstants.FrontRight,
                new ModuleIO() {
                },
                TestBotTunerConstants.BackLeft,
                new ModuleIO() {
                },
                TestBotTunerConstants.BackRight
        );
    }

    default Feeder createFeeder() {
        return new Feeder(new FeederIO() {
        });
    }

    default Hopper createHopper() {
        return new Hopper(new HopperIO() {
        });
    }

    default Intake createIntake() {
        return new Intake(new IntakeIO() {
        });
    }

    default Shooter createShooter() {
        return new Shooter(new ShooterIO() {
        });
    }
}
