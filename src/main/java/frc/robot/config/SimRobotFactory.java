package frc.robot.config;

import frc.robot.generated.C2026TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.SimFeederIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.SimHopperIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.SimIntakeIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.SimShooterIO;

public class SimRobotFactory implements RobotFactory {
    @Override
    public Drive createDrive() {
        // Sim robot, instantiate physics sim IO implementations
        return new Drive(
                new GyroIO() {
                },
                new ModuleIOSim(C2026TunerConstants.FrontLeft),
                C2026TunerConstants.FrontLeft,
                new ModuleIOSim(C2026TunerConstants.FrontRight),
                C2026TunerConstants.FrontRight,
                new ModuleIOSim(C2026TunerConstants.BackLeft),
                C2026TunerConstants.BackLeft,
                new ModuleIOSim(C2026TunerConstants.BackRight),
                C2026TunerConstants.BackRight);
    }

    @Override
    public Feeder createFeeder() {
        return new Feeder(new SimFeederIO());
    }

    @Override
    public Hopper createHopper() {
        return new Hopper(new SimHopperIO());
    }

    @Override
    public Intake createIntake() {
        return new Intake(new SimIntakeIO());
    }

    @Override
    public Shooter createShooter() {
        return new Shooter(new SimShooterIO());
    }
}
