package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026HopperIO implements HopperIO {
    private final TalonFX motor;
    private final TalonFX corneringMotor;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2026HopperIO(TalonFX motor, TalonFX corneringMotor) {
        this.motor = motor;
        this.corneringMotor = corneringMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        motor.getConfigurator().apply(config);
        corneringMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.curentVelocity = motor.getVelocity().getValueAsDouble();
        inputs.currentAppliedVoltage = motor.getSupplyVoltage().getValueAsDouble();

        inputs.motorTemperature = motor.getDeviceTemp().getValueAsDouble();
        inputs.motorSupplyCurrent = motor.getSupplyCurrent().getValueAsDouble();

        inputs.corneringMotorCurentVelocity = corneringMotor.getVelocity().getValueAsDouble();
        inputs.corneringMotorCurrentAppliedVoltage = corneringMotor.getSupplyVoltage().getValueAsDouble();

        inputs.corneringMotorTemperature = corneringMotor.getDeviceTemp().getValueAsDouble();
        inputs.corneringMotorSupplyCurrent = corneringMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetVoltage(double voltage, double corneringVoltage) {
        motor.setControl(voltageRequest.withOutput(voltage));
        corneringMotor.setControl(voltageRequest.withOutput(corneringVoltage));
    }

    @Override
    public void stop() {
        motor.setControl(stopRequest);
        corneringMotor.setControl(stopRequest);
    }
}