package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026HopperIO implements HopperIO {
    private static final double TUNNEL_GEAR_RATIO = 1.0 / 2.0;

    private final TalonFX hopperMotor;
    private final TalonFX upperTunnelMotor;
    private final TalonFX lowerTunnelMotor;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2026HopperIO(TalonFX hopperMotor, TalonFX upperTunnelMotor, TalonFX lowerTunnelMotor) {
        this.hopperMotor = hopperMotor;
        this.upperTunnelMotor = upperTunnelMotor;
        this.lowerTunnelMotor = lowerTunnelMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        TalonFXConfiguration upperTunnelConfig = new TalonFXConfiguration();
        upperTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        upperTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimit = 20;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperTunnelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        TalonFXConfiguration lowerTunnelConfig = new TalonFXConfiguration();
        lowerTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        lowerTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimit = 20;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lowerTunnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        hopperMotor.getConfigurator().apply(config);
        upperTunnelMotor.getConfigurator().apply(upperTunnelConfig);
        lowerTunnelMotor.getConfigurator().apply(lowerTunnelConfig);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.curentHopperVelocity = hopperMotor.getVelocity().getValueAsDouble();
        inputs.currentHopperAppliedVoltage = hopperMotor.getMotorVoltage().getValueAsDouble();

        inputs.hopperTemperature = hopperMotor.getDeviceTemp().getValueAsDouble();
        inputs.hopperSupplyCurrent = hopperMotor.getSupplyCurrent().getValueAsDouble();

        inputs.curentUpperTunnelVelocity = upperTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentUpperTunnelAppliedVoltage = upperTunnelMotor.getMotorVoltage().getValueAsDouble();

        inputs.upperTunnelTemperature = upperTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.upperTunnelSupplyCurrent = upperTunnelMotor.getSupplyCurrent().getValueAsDouble();

        inputs.curentLowerTunnelVelocity = lowerTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentLowerTunnelAppliedVoltage = lowerTunnelMotor.getMotorVoltage().getValueAsDouble();

        inputs.lowerTunnelTemperature = lowerTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.lowerTunnelSupplyCurrent = lowerTunnelMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetVoltage(double tunnelVoltage, double hopperVoltage) {
        hopperMotor.setControl(voltageRequest.withOutput(hopperVoltage));
        upperTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
        lowerTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
    }

    @Override
    public void stop() {
        hopperMotor.setControl(stopRequest);
        upperTunnelMotor.setControl(stopRequest);
        lowerTunnelMotor.setControl(stopRequest);
    }
}