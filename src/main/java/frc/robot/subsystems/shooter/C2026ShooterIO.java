package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;

public class C2026ShooterIO implements ShooterIO {
    // TODO: get actual values for these constants
    private static final Shooter.Constants constants = new Shooter.Constants(
            0, Units.degreesToRadians(90), Units.degreesToRadians(-180), Units.degreesToRadians(180));
    private static final double HOOD_GEAR_RATIO = 1.0;
    private static final double TURRET_GEAR_RATIO = 1.0;
    private static final double SHOOTER_GEAR_RATIO = 1.0;
    private static final double SHOOTER_WHEEL_RADIUS = Units.inchesToMeters(2.0);

    // Gear Ratios
    // TODO get actual gear ratio for ENCODER_TO_MECHANISM
    private static final double MOTOR_TO_MECHANISM_RATIO = (80.0 / 10.0);
    private static final double MOTOR_TO_ENCODER_RATIO = (60.0 / 18.0) * (48.0 / 16.0);
    private static final double ENCODER_TO_MECHANISM_RATIO = MOTOR_TO_MECHANISM_RATIO / MOTOR_TO_ENCODER_RATIO;
    private static final double ENCODER_OFFSET = -0.3779;

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

        // Hood motor config
        TalonFXConfiguration hoodConfig = new TalonFXConfiguration();
        hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hoodConfig.Feedback.SensorToMechanismRatio = HOOD_GEAR_RATIO;
        hoodConfig.CurrentLimits.SupplyCurrentLimit = 60;
        hoodConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        hoodConfig.Slot0.kV = 0.0;
        hoodConfig.Slot0.kA = 0.0;
        hoodConfig.Slot0.kP = 0.0;
        hoodConfig.Slot0.kI = 0.0;
        hoodConfig.Slot0.kD = 0.0;

//        shooterConfig.MotionMagic.MotionMagicJerk = 15.0;
//        shooterConfig.MotionMagic.MotionMagicAcceleration = 5.0;
//        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;

        hoodMotor.getConfigurator().apply(hoodConfig);

        // Turret motor config
        TalonFXConfiguration turretConfig = new TalonFXConfiguration();
        turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turretConfig.CurrentLimits.SupplyCurrentLimit = 60;
        turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        turretConfig.Feedback.FeedbackRemoteSensorID = turretCanCoder.getDeviceID();
        turretConfig.Feedback.SensorToMechanismRatio = ENCODER_TO_MECHANISM_RATIO;
        turretConfig.Feedback.RotorToSensorRatio = MOTOR_TO_ENCODER_RATIO;
        turretConfig.MotorOutput.PeakForwardDutyCycle = 0.1;
        turretConfig.MotorOutput.PeakReverseDutyCycle = -0.1;

        turretConfig.Slot0.kV = 0.0;
        turretConfig.Slot0.kA = 0.0;
        turretConfig.Slot0.kP = 0.0;
        turretConfig.Slot0.kI = 0.0;
        turretConfig.Slot0.kD = 0.0;

//        shooterConfig.MotionMagic.MotionMagicJerk = 15.0;
//        shooterConfig.MotionMagic.MotionMagicAcceleration = 5.0;
//        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;

        turretMotor.getConfigurator().apply(turretConfig);

        // Shooter motors config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Feedback.RotorToSensorRatio = SHOOTER_GEAR_RATIO;
        shooterConfig.CurrentLimits.SupplyCurrentLimit = 60;
        shooterConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        shooterConfig.Slot0.kV = 0.0;
        shooterConfig.Slot0.kA = 0.0;
        shooterConfig.Slot0.kP = 0.0;
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
        inputs.currentTurretPosition = Units.rotationsToRadians(turretMotor.getPosition().getValueAsDouble());
        inputs.currentTurretAppliedVoltage = turretMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentTurretVelocity = Units.rotationsToRadians(turretMotor.getVelocity().getValueAsDouble());
        inputs.turretMotorTemperature = turretMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentTurretDraw = turretMotor.getSupplyCurrent().getValueAsDouble();

        inputs.currentTurretEncoderPosition = turretCanCoder.getPosition().getValueAsDouble();

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

//    @Override
//    public void stopHood() {
//        hoodMotor.setControl(stopRequest);
//    }
//
//    @Override
//    public void resetHoodPosition(double position) {
//        hoodMotor.setPosition(Units.radiansToRotations(position));
//    }
//
//    @Override
//    public void setHoodTargetPosition(double position) {
//        hoodMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
//    }
//
//    @Override
//    public void setHoodTargetVoltage(double voltage) {
//        hoodMotor.setControl(voltageRequest.withOutput(voltage));
//    }

    @Override
    public void stopTurret() {
        turretMotor.setControl(stopRequest);
    }

    @Override
    public void setTurretTargetPosition(double position) {
        turretMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTurretTargetVoltage(double voltage) {
        turretMotor.setControl(voltageRequest.withOutput(voltage));
    }

//    @Override
//    public void stopShooter() {
//        shooterMotor.setControl(stopRequest);
//    }
//
//    @Override
//    public void setShooterTargetVelocity(double velocity) {
//        shooterMotor.setControl(velocityRequest.withVelocity(velocity));
//    }
//
//    @Override
//    public void setShooterTargetVoltage(double voltage) {
//        shooterMotor.setControl(voltageRequest.withOutput(voltage));
//    }
}