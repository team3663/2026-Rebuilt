package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.hood.C2026HoodIO;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class C2026TurretIO implements TurretIO{
    private final Turret.Constants CONSTANTS = new Turret.Constants(Units.degreesToRadians(-175.0),
            Units.degreesToRadians(190.0));
    private static final double MOTOR_TO_MECHANISM_RATIO = (42.0 / 18.0) * (80.0 / 16.0);
    private static final double MOTOR_TO_SENSOR_RATIO = (42.0 / 18.0) * (46.0 / 16.0) * (60.0 / 24.0);
    private static final double SENSOR_TO_MECHANISM_RATIO = MOTOR_TO_MECHANISM_RATIO / MOTOR_TO_SENSOR_RATIO;
    private static final double ENCODER_OFFSET = 0.1796875;

    private final TalonFX turret;
    private final CANcoder turretCanCoder;

    private NeutralOut stopRequest = new NeutralOut();
    private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);


    public C2026TurretIO(TalonFX turret, CANcoder turretCanCoder) {
        this.turret = turret;
        this.turretCanCoder = turretCanCoder;

        // CANCoder config
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = ENCODER_OFFSET;

        tryUntilOk(5, () -> turretCanCoder.getConfigurator().apply(canCoderConfig, 0.25));

        // Turret motor config
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 60;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM_RATIO;
        turretConfig.Feedback.RotorToSensorRatio = MOTOR_TO_SENSOR_RATIO;
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turretConfig.Feedback.FeedbackRemoteSensorID = turretCanCoder.getDeviceID();

        turretConfig.Slot0.kV = 12 / ((7368.0 / 60.0) * MOTOR_TO_MECHANISM_RATIO);
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kP = 80.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = 7.5;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0;

        tryUntilOk(5, () -> turret.getConfigurator().apply(turretConfig, 0.25));
    }


}
