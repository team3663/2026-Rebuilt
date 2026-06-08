package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026FeederIO implements FeederIO{
    private static final Feeder.Constants CONSTANTS = new Feeder.Constants(6.0, -3.0);

    private final TalonFX feeder;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2026FeederIO(TalonFX feeder) {
        this.feeder = feeder;

        // Configuring the feeder motor
        TalonFXConfiguration feederConfig = new TalonFXConfiguration();
        feederConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        feederConfig.CurrentLimits.SupplyCurrentLimit = 60;
        feederConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        feeder.getConfigurator().apply(feederConfig);
    }

    @Override
    public void updateInputs(FeederInputs inputs) {
        inputs.currentFeederVelocity = feeder.getVelocity().getValueAsDouble();
        inputs.currentFeederAppliedVoltage = feeder.getMotorVoltage().getValueAsDouble();
        inputs.feederCurrentDraw = feeder.getSupplyCurrent().getValueAsDouble();
        inputs.feederMotorTemperature = feeder.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public void stop(){
        feeder.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double volts){
        feeder.setControl(voltageRequest.withOutput(volts));
    }

    public Feeder.Constants getConstants() {
        return CONSTANTS;
    }

}
