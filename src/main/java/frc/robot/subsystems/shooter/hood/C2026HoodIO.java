package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class C2026HoodIO implements HoodIO{
    private static final Hood.Constants CONSTANTS = new Hood.Constants(Units.degreesToRadians(0.0),
            Units.degreesToRadians(16.5));

    private static final double HOOD_GEAR_RATIO = 340.0 / 14.0;

    private final TalonFX hoodMotor;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2026HoodIO(TalonFX hoodMotor) {
        this.hoodMotor = hoodMotor;

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

        tryUntilOk(5, () -> hoodMotor.getConfigurator().apply(hoodConfig, 0.25));
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        // Hood Motor
        inputs.currentHoodAppliedVoltage = hoodMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentHoodVelocity = Units.rotationsToRadians(hoodMotor.getVelocity().getValueAsDouble());
        inputs.currentHoodPosition = Units.rotationsToRadians(hoodMotor.getPosition().getValueAsDouble());
        inputs.hoodMotorTemperature = hoodMotor.getDeviceTemp().getValueAsDouble();
        inputs.hoodCurrentDraw = hoodMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void stop(){
        hoodMotor.setControl(stopRequest);
    }

    @Override
    public void setVoltage(double volts) {
        hoodMotor.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void setPosition(double angle) {
        hoodMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(angle)));
    }

    @Override
    public void resetPosition(double position) {
        hoodMotor.setPosition(Units.radiansToRotations(position));
    }

}
