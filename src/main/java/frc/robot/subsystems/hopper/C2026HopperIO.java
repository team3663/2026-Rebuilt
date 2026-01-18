package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026HopperIO implements HopperIO {
    private final TalonFX motor;

    public C2026HopperIO(TalonFX motor) {
        this.motor = motor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.curentVelocity = motor.getVelocity().getValueAsDouble();
        inputs.currentAppliedVoltage = motor.getSupplyVoltage().getValueAsDouble();

        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void stop() {
        motor.setVoltage(0.0);
    }
}