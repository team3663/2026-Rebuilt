package frc.robot.config;

import frc.robot.generated.TestBotTunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

public class TestBotRobotFactory implements RobotFactory {
    @Override
    public Drive createDrive() {
        double odometryFrequencyHz = TestBotTunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
        PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(odometryFrequencyHz, TestBotTunerConstants.kCANBus);

        Drive drive = new Drive(
                new GyroIOPigeon2(odometryThread, odometryFrequencyHz, TestBotTunerConstants.DrivetrainConstants.Pigeon2Id,
                        TestBotTunerConstants.DrivetrainConstants.Pigeon2Configs, TestBotTunerConstants.kCANBus),
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, TestBotTunerConstants.FrontLeft, TestBotTunerConstants.kCANBus),
                TestBotTunerConstants.FrontLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, TestBotTunerConstants.FrontRight, TestBotTunerConstants.kCANBus),
                TestBotTunerConstants.FrontRight,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, TestBotTunerConstants.BackLeft, TestBotTunerConstants.kCANBus),
                TestBotTunerConstants.BackLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, TestBotTunerConstants.BackRight, TestBotTunerConstants.kCANBus),
                TestBotTunerConstants.BackRight);

        // Start odometry thread
        odometryThread.start();

        return drive;
    }
}
