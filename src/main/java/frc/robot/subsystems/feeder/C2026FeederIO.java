package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.reusablecode.ReusableCommands;

public class C2026FeederIO implements FeederIO {
    // Feeder Motor
    private final TalonFX motor; //ID 14;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
    private final ReusableCommands reusableCommands = new ReusableCommands();
    public C2026FeederIO(TalonFX motor) {
        this.motor = motor;

        TalonFXConfiguration motor1Config = new TalonFXConfiguration();
        reusableCommands.configurations(motor1Config, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake, 60, true);
        motor.getConfigurator().apply(motor1Config);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.currentVelocity = motor.getVelocity().getValueAsDouble();
        inputs.currentAppliedVoltage = motor.getMotorVoltage().getValueAsDouble();
        inputs.feederCurrentDraw = motor.getSupplyCurrent().getValueAsDouble();
        inputs.feederTemperature = motor.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetVelocity(double velocity) {
        motor.setControl(velocityVoltage.withVelocity(velocity));
    }
}