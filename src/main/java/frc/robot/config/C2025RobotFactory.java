package frc.robot.config;

import com.ctre.phoenix6.hardware.CANdle;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.generated.C2025TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.PhoenixOdometryThread;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedCandleIo;
import frc.robot.subsystems.vision.LimelightIO;
import frc.robot.subsystems.vision.Vision;

public class C2025RobotFactory implements RobotFactory {
    @Override
    public Drive createDrive() {
        double odometryFrequencyHz = C2025TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
        PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(odometryFrequencyHz, C2025TunerConstants.kCANBus);

        Drive drive = new Drive(
                new GyroIOPigeon2(odometryThread, odometryFrequencyHz, C2025TunerConstants.DrivetrainConstants.Pigeon2Id,
                        C2025TunerConstants.DrivetrainConstants.Pigeon2Configs, C2025TunerConstants.kCANBus),
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2025TunerConstants.FrontLeft, C2025TunerConstants.kCANBus),
                C2025TunerConstants.FrontLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2025TunerConstants.FrontRight, C2025TunerConstants.kCANBus),
                C2025TunerConstants.FrontRight,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2025TunerConstants.BackLeft, C2025TunerConstants.kCANBus),
                C2025TunerConstants.BackLeft,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, C2025TunerConstants.BackRight, C2025TunerConstants.kCANBus),
                C2025TunerConstants.BackRight);

        // Start odometry thread
        odometryThread.start();

        return drive;
    }

    @Override
    public Vision createVision() {
        Rotation3d frontLeftRotation = new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(0.0477));
        Transform3d frontLeftTransform = new Transform3d(Units.inchesToMeters(27.0 / 2 - 5.0),
                Units.inchesToMeters(27.0 / 2.0 - 5.3), Units.inchesToMeters(8.8125),
                frontLeftRotation);

        Rotation3d backRotation = new Rotation3d(0.0, Units.degreesToRadians(20.0), Units.degreesToRadians(200.0));
        Transform3d backTransform = new Transform3d(Units.inchesToMeters(27.0 / 2.0 - 4.625), Units.inchesToMeters(27.0 / 2.0 - 5.125),
                Units.inchesToMeters(8.375), backRotation);

        return new Vision (AprilTagFieldLayout.loadField(Constants.IS_ANDYMARK ? AprilTagFields.k2026RebuiltAndymark : AprilTagFields.k2026RebuiltWelded),
                new LimelightIO("limelight", frontLeftTransform, false),
                new LimelightIO("limelight-back", backTransform, false)
        );
    }
}