package frc.robot.config;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.PhoenixOdometryThread;

public class C2025RobotFactory implements RobotFactory {
    private static final CANBus DRIVETRAIN_CAN_BUS = new CANBus("3663");

    public static final double MK4_2PLUS_REDUCTION = (50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double MK4N_STEER_REDUCTION = 18.75;
    public static final double MK4N_STEER_INERTIA = 0.00001;
    public static final double MK4N_STEER_FRICTION_VOLTAGE = 0.25;
    public static final Slot0Configs MK4N_STEER_PID_CONSTANTS = new Slot0Configs()
            .withKP(100.0);
    public static final double MK4_WHEEL_RADIUS = Units.inchesToMeters(1.93);

    private static final double MODULE_WHEEL_INSET = Units.inchesToMeters(2.625);
    private static final double FRAME_X_LENGTH = Units.inchesToMeters(27.0);
    private static final double FRAME_Y_LENGTH = Units.inchesToMeters(27.0);
    private static final double MODULE_X_OFFSET = FRAME_X_LENGTH / 2.0 - MODULE_WHEEL_INSET;
    private static final double MODULE_Y_OFFSET = FRAME_Y_LENGTH / 2.0 - MODULE_WHEEL_INSET;

    private static final MountPoseConfigs mountPose= new MountPoseConfigs();
    private static final  GyroTrimConfigs gyroTrim= new GyroTrimConfigs();

    private static final SwerveDrivetrainConstants DRIVETRAIN_CONSTANTS = new SwerveDrivetrainConstants()
            .withCANBusName(DRIVETRAIN_CAN_BUS.getName())
            .withPigeon2Id(1)
            .withPigeon2Configs(new Pigeon2Configuration().withMountPose(mountPose).withGyroTrim(gyroTrim));


    private static final double DRIVE_INERTIA = 0.01;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;
    private static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(100).withStatorCurrentLimitEnable(true));
    private static final Slot0Configs DRIVE_PID_CONSTANTS = new Slot0Configs()
            .withKS(0.24802)
            .withKA(0.0081758)
            .withKV(0.1239)
            .withKP(0.18498);
    private static final TalonFXConfiguration STEER_CONFIG = new TalonFXConfiguration();
    private static final double MAX_DRIVE_VELOCITY = DCMotor.getFalcon500Foc(1)
            .freeSpeedRadPerSec / MK4_2PLUS_REDUCTION * MK4_WHEEL_RADIUS;

    // Creating a constants factory for the drive and steer motors of the drivetrain
    private static final SwerveModuleConstantsFactory<
                TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> MODULE_CONSTANTS_FACTORY
            = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
            .withDriveMotorGearRatio(MK4_2PLUS_REDUCTION)
            .withDriveInertia(DRIVE_INERTIA)
            .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
            .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withDriveMotorInitialConfigs(DRIVE_CONFIG)
            .withDriveMotorGains(DRIVE_PID_CONSTANTS)
            .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
            .withSteerMotorGearRatio(MK4N_STEER_REDUCTION)
            .withSteerInertia(MK4N_STEER_INERTIA)
            .withSteerFrictionVoltage(MK4N_STEER_FRICTION_VOLTAGE)
            .withSteerMotorInitialConfigs(STEER_CONFIG)
            .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
            .withSteerMotorGains(MK4N_STEER_PID_CONSTANTS)
            .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
            .withWheelRadius(MK4_WHEEL_RADIUS)
            .withSpeedAt12Volts(MAX_DRIVE_VELOCITY);


    // offsets are found with gears to the right
    // Front Left
    private static final int DRIVETRAIN_FRONT_LEFT_STEER_ID = 2;
    private static final int DRIVETRAIN_FRONT_LEFT_DRIVE_ID = 1;
    private static final int DRIVETRAIN_FRONT_LEFT_ENCODER_ID = 2;
    private static final double DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET = Units.degreesToRotations(-266.8359375);

    // Front Right
    private static final int DRIVETRAIN_FRONT_RIGHT_STEER_ID = 4;
    private static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_ID = 3;
    private static final int DRIVETRAIN_FRONT_RIGHT_ENCODER_ID = 4;
    private static final double DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-153.6328125);

    // Back Left
    private static final int DRIVETRAIN_BACK_LEFT_STEER_ID = 6;
    private static final int DRIVETRAIN_BACK_LEFT_DRIVE_ID = 5;
    private static final int DRIVETRAIN_BACK_LEFT_ENCODER_ID = 6;
    private static final double DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET = Units.degreesToRotations(70.48828125);

    // Back Right
    private static final int DRIVETRAIN_BACK_RIGHT_STEER_ID = 8;
    private static final int DRIVETRAIN_BACK_RIGHT_DRIVE_ID = 7;
    private static final int DRIVETRAIN_BACK_RIGHT_ENCODER_ID = 8;
    private static final double DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET = Units.degreesToRotations(-179.296875);

    @Override
    public Drive createDrive() {
        // Configuring front left module
        var frontLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_LEFT_STEER_ID,
                DRIVETRAIN_FRONT_LEFT_DRIVE_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_ID,
                DRIVETRAIN_FRONT_LEFT_ENCODER_OFFSET,
                MODULE_X_OFFSET, MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring front right module
        var frontRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_FRONT_RIGHT_STEER_ID,
                DRIVETRAIN_FRONT_RIGHT_DRIVE_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_ID,
                DRIVETRAIN_FRONT_RIGHT_ENCODER_OFFSET,
                MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring back left module
        var backLeftConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_LEFT_STEER_ID,
                DRIVETRAIN_BACK_LEFT_DRIVE_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_ID,
                DRIVETRAIN_BACK_LEFT_ENCODER_OFFSET,
                -MODULE_X_OFFSET, MODULE_Y_OFFSET,
                false, true, false
        );

        // Configuring back right module
        var backRightConfig = MODULE_CONSTANTS_FACTORY.createModuleConstants(
                DRIVETRAIN_BACK_RIGHT_STEER_ID,
                DRIVETRAIN_BACK_RIGHT_DRIVE_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_ID,
                DRIVETRAIN_BACK_RIGHT_ENCODER_OFFSET,
                -MODULE_X_OFFSET, -MODULE_Y_OFFSET,
                false, true, false
        );

        double odometryFrequencyHz = DRIVETRAIN_CAN_BUS.isNetworkFD() ? 250.0 : 100.0;
        PhoenixOdometryThread odometryThread = new PhoenixOdometryThread(odometryFrequencyHz, DRIVETRAIN_CAN_BUS);

        Drive drive = new Drive(
                new GyroIOPigeon2(odometryThread, odometryFrequencyHz, DRIVETRAIN_CONSTANTS.Pigeon2Id,
                        DRIVETRAIN_CONSTANTS.Pigeon2Configs, DRIVETRAIN_CAN_BUS),
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, frontLeftConfig, DRIVETRAIN_CAN_BUS),
                frontLeftConfig,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, frontRightConfig, DRIVETRAIN_CAN_BUS),
                frontRightConfig,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, backLeftConfig, DRIVETRAIN_CAN_BUS),
                backLeftConfig,
                new ModuleIOTalonFX(odometryThread, odometryFrequencyHz, backRightConfig, DRIVETRAIN_CAN_BUS),
                backRightConfig);

        // Start odometry thread
        odometryThread.start();

        return drive;
    }
}
