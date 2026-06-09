package frc.robot.subsystems.shooter.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.shooter.ShooterInputs;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class C2026ShooterIO implements ShooterIO{
    // TODO - SET DEFAULT VELOCITY
    private static final Shooter.Constants CONSTANTS = new Shooter.Constants(0.0);
    private static final double SHOOTER_GEAR_RATIO = (18.0 / 15.0);

    private final TalonFX shooter;
    private final TalonFX shooter2;

    private static final CurrentLimitsConfigs currentLimitsEnabled = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(10.0)
            .withSupplyCurrentLimitEnable(true);

    // Motor Requests
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2026ShooterIO(TalonFX shooter, TalonFX shooter2) {
        this.shooter = shooter;
        this.shooter2 = shooter2;

        // Shooter motors config
        TalonFXConfiguration shooterConfig = new TalonFXConfiguration();
        shooterConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        shooterConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        shooterConfig.Feedback.RotorToSensorRatio = SHOOTER_GEAR_RATIO;
        shooterConfig.CurrentLimits = currentLimitsEnabled;

        shooterConfig.Slot0.kV = 12 / ((7758.0 / 60.0) * SHOOTER_GEAR_RATIO);
        shooterConfig.Slot0.kA = 0.0;
        shooterConfig.Slot0.kP = 0.5;
        shooterConfig.Slot0.kI = 0.0;
        shooterConfig.Slot0.kD = 0.0;

//        shooterConfig.MotionMagic.MotionMagicJerk = 15.0;
//        shooterConfig.MotionMagic.MotionMagicAcceleration = 5.0;
//        shooterConfig.MotionMagic.MotionMagicCruiseVelocity = 2.0;

        tryUntilOk(5, () -> shooter.getConfigurator().apply(shooterConfig, 0.25));
        tryUntilOk(5, () -> shooter2.getConfigurator().apply(shooterConfig, 0.25));

        shooter2.setControl(new Follower(shooter.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        // Shooter Motor 1
        inputs.currentShooterAppliedVoltage1 = shooter.getMotorVoltage().getValueAsDouble();
        inputs.currentShooterVelocity1 = Units.rotationsToRadians(shooter.getVelocity().getValueAsDouble());
        inputs.shooterMotorTemperature1 = shooter.getDeviceTemp().getValueAsDouble();
        inputs.currentShooterDraw1 = shooter.getSupplyCurrent().getValueAsDouble();

        // Shooter Motor 2
        inputs.currentShooterAppliedVoltage2 = shooter2.getMotorVoltage().getValueAsDouble();
        inputs.currentShooterVelocity2 = Units.rotationsToRadians(shooter2.getVelocity().getValueAsDouble());
        inputs.shooterMotorTemperature2 = shooter2.getDeviceTemp().getValueAsDouble();
        inputs.currentShooterDraw2 = shooter2.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public Shooter.Constants getConstants() {
        return CONSTANTS;
    }


}
