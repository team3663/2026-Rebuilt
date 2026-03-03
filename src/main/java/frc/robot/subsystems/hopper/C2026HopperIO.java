package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026HopperIO implements HopperIO {
    private static final double CORNERING_GEAR_RATIO = 12.0 / 18.0;

    private final TalonFX hopperMotor;
    private final TalonFX corneringMotor;
    private final TalonFX indexingMotor;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2026HopperIO(TalonFX hopperMotor, TalonFX corneringMotor, TalonFX indexingMotor) {
        this.hopperMotor = hopperMotor;
        this.corneringMotor = corneringMotor;
        this.indexingMotor = indexingMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        TalonFXConfiguration corneringConfig = new TalonFXConfiguration();
        corneringConfig.Feedback.SensorToMechanismRatio = CORNERING_GEAR_RATIO;
        corneringConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        corneringConfig.CurrentLimits.SupplyCurrentLimit = 20;
        corneringConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        corneringConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hopperMotor.getConfigurator().apply(config);
        corneringMotor.getConfigurator().apply(corneringConfig);
        indexingMotor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.curentVelocity = hopperMotor.getVelocity().getValueAsDouble();
        inputs.currentAppliedVoltage = hopperMotor.getSupplyVoltage().getValueAsDouble();

        inputs.motorTemperature = hopperMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorSupplyCurrent = hopperMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetVoltage(double voltage) {
        hopperMotor.setControl(voltageRequest.withOutput(voltage));
        corneringMotor.setControl(voltageRequest.withOutput(voltage * (1.5)));
        indexingMotor.setControl(voltageRequest.withOutput(voltage * (-1.5)));
    }

    @Override
    public void stop() {
        hopperMotor.setControl(stopRequest);
        corneringMotor.setControl(stopRequest);
        indexingMotor.setControl(stopRequest);
    }
}