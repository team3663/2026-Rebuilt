package frc.robot.config;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedIo;
import frc.robot.subsystems.shooter.ShooterSuperStructure;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.shooter.Shooter;
import frc.robot.subsystems.shooter.shooter.ShooterIO;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;

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

    default ShooterSuperStructure createShooter() {
        return new ShooterSuperStructure(
                new Shooter(new ShooterIO() {}),
                new Hood(new HoodIO() {}),
                new Turret(new TurretIO() {}));
    }

    default Vision createVision() {
        return new Vision(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField), new VisionIO() {});
    }

    default Led createLed() {
        return new Led(new LedIo() {
        });
    }
}
