package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;

public class C2026ShooterIO implements ShooterIO {
    private static final Shooter.Constants constants = new Shooter.Constants(
            0.0,
            Units.degreesToRadians(18.0),
            Units.degreesToRadians(-185.0),
            Units.degreesToRadians(165.0));
    private static final double HOOD_GEAR_RATIO = 340.0 / 14.0;
    private static final double SHOOTER_GEAR_RATIO = (15.0 / 18.0);

    // Gear Ratios
    private static final double MOTOR_TO_MECHANISM_RATIO = (44.0 / 14.0) * (80.0 / 16.0);
    private static final double MOTOR_TO_SENSOR_RATIO = (44.0 / 14.0) * (62.0 / 18.0) * (60.0 / 24.0);
    private static final double SENSOR_TO_MECHANISM_RATIO = MOTOR_TO_MECHANISM_RATIO / MOTOR_TO_SENSOR_RATIO;
    private static final double ENCODER_OFFSET = -0.1845703125;

    private final TalonFX hoodMotor;
    private final TalonFX turretMotor;
    private final CANcoder turretCanCoder;
    private final TalonFX shooterMotor;
    private final TalonFX shooterMotor2;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2026ShooterIO(TalonFX hoodMotor, TalonFX turretMotor, TalonFX shooterMotor, TalonFX shooterMotor2,
                          CANcoder turretCanCoder) {
        this.hoodMotor = hoodMotor;
        this.turretMotor = turretMotor;
        this.turretCanCoder = turretCanCoder;
        this.shooterMotor = shooterMotor;
        this.shooterMotor2 = shooterMotor2;

        // TODO: get actual values for all the configs
        // CANCoder config
        CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
        canCoderConfig.MagnetSensor.MagnetOffset = ENCODER_OFFSET;

        turretCanCoder.getConfigurator().apply(canCoderConfig);

        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

        // Hood motor config
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 60;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0;
        hoodConfig.MotionMagic.MotionMagicAcceleration = 10.0;

        hoodConfig.Slot0.kV = 12 / ((7368.0 / 60.0) * HOOD_GEAR_RATIO);
        hoodConfig.Slot0.kA = 0.0;
        hoodConfig.Slot0.kP = 300.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;
        hoodConfig.Slot0.kS = 0.8;

        hoodMotor.getConfigurator().apply(hoodConfig);

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
        turretConfig.Slot0.kP = 100.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;

        turretConfig.MotionMagic.MotionMagicAcceleration = 7.5;
        turretConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0;

        turretMotor.getConfigurator().apply(turretConfig);

        // Shooter motors config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Feedback.RotorToSensorRatio = SHOOTER_GEAR_RATIO;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.Slot0.kV = 12 / ((7368.0 / 60.0) * SHOOTER_GEAR_RATIO);
        shooterConfig.Slot0.kA = 0.0;
        shooterConfig.Slot0.kP = 0.1;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;

//        shooterConfig.MotionMagic.MotionMagicJerk = 15.0;
//        shooterConfig.MotionMagic.MotionMagicAcceleration = 5.0;
//        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;

        shooterMotor.getConfigurator().apply(shooterConfig);
        shooterMotor2.getConfigurator().apply(shooterConfig);

        shooterMotor2.setControl(new Follower(shooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public Shooter.Constants getConstants() {
        return constants;
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        // Hood Motor
        inputs.currentHoodAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentHoodVelocity = Units.rotationsToRadians(hoodMotor.getVelocity().getValueAsDouble());
        inputs.currentHoodPosition = Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
        inputs.hoodMotorTemperature = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentHoodDraw = hoodMotor.getSupplyCurrent().getValueAsDouble();

        // Turret
        inputs.currentTurretAppliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentTurretVelocity = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble());
        inputs.turretMotorTemperature = turretMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentTurretDraw = turretMotor.getSupplyCurrent().getValueAsDouble();

        inputs.currentTurretEncoderPosition = turretCanCoder.getPosition().getValueAsDouble();

        inputs.currentTurretPosition = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());

        // Shooter Motor 1
        inputs.currentShooterAppliedVoltage1 = shooterMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentShooterVelocity1 = Units.rotationsToRadians(shooterMotor.getVelocity().getValueAsDouble());
        inputs.shooterMotorTemperature1 = shooterMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentShooterDraw1 = shooterMotor.getSupplyCurrent().getValueAsDouble();

        // Shooter Motor 2
        inputs.currentShooterAppliedVoltage2 = shooterMotor2.getMotorVoltage().getValueAsDouble();
        inputs.currentShooterVelocity2 = Units.rotationsToRadians(shooterMotor2.getVelocity().getValueAsDouble());
        inputs.shooterMotorTemperature2 = shooterMotor2.getDeviceTemp().getValueAsDouble();
        inputs.currentShooterDraw2 = shooterMotor2.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void stopHood() {
        hoodMotor.setControl(stopRequest);
    }

    @Override
    public void resetHoodPosition(double position) {
        hoodMotor.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setHoodTargetPosition(double position) {
        hoodMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setHoodTargetVoltage(double voltage) {
        hoodMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stopTurret() {
        turretMotor.setControl(stopRequest);
    }

    @Override
    public void setTurretTargetPosition(double position) {
        turretMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(MathUtil.clamp(position, constants.minimumTurretPosition(), constants.maximumTurretPosition()))));
    }

    @Override
    public void setTurretTargetVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void stopShooter() {
        shooterMotor.setControl(stopRequest);
    }

    @Override
    public void setShooterTargetVelocity(double velocity) {
        shooterMotor.setControl(velocityRequest.withVelocity(Units.radiansToRotations(velocity)));
    }

    @Override
    public void setShooterTargetVoltage(double voltage) {
        shooterMotor.setControl(voltageRequest.withOutput(voltage));
    }
}