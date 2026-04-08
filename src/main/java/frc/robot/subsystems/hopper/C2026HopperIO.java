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
    private final TalonFX topRollerMotor;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);

    public C2026HopperIO(TalonFX hopperMotor, TalonFX upperTunnelMotor, TalonFX lowerTunnelMotor, TalonFX topRollerMotor) {
        this.hopperMotor = hopperMotor;
        this.upperTunnelMotor = upperTunnelMotor;
        this.lowerTunnelMotor = lowerTunnelMotor;
        this.topRollerMotor = topRollerMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.CurrentLimits.SupplyCurrentLimit = 20;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        TalonFXConfiguration upperTunnelConfig = new TalonFXConfiguration();
        upperTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        upperTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimit = 30;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperTunnelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        TalonFXConfiguration lowerTunnelConfig = new TalonFXConfiguration();
        lowerTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        lowerTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimit = 30;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lowerTunnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        TalonFXConfiguration topRollerConfig = new TalonFXConfiguration();
        topRollerConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        topRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        hopperMotor.getConfigurator().apply(config);
        upperTunnelMotor.getConfigurator().apply(upperTunnelConfig);
        lowerTunnelMotor.getConfigurator().apply(lowerTunnelConfig);
        topRollerMotor.getConfigurator().apply(topRollerConfig);


        topRollerMotor.getSupplyCurrent()
                .setUpdateFrequency(50.0);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        inputs.currentHopperVelocity = hopperMotor.getVelocity().getValueAsDouble();
        inputs.currentHopperAppliedVoltage = hopperMotor.getMotorVoltage().getValueAsDouble();
        inputs.hopperTemperature = hopperMotor.getDeviceTemp().getValueAsDouble();
        inputs.hopperCurrentDraw = hopperMotor.getSupplyCurrent().getValueAsDouble();

        inputs.currentUpperTunnelVelocity = upperTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentUpperTunnelAppliedVoltage = upperTunnelMotor.getMotorVoltage().getValueAsDouble();
        inputs.upperTunnelTemperature = upperTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.upperTunnelCurrentDraw = upperTunnelMotor.getSupplyCurrent().getValueAsDouble();

        inputs.currentLowerTunnelVelocity = lowerTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentLowerTunnelAppliedVoltage = lowerTunnelMotor.getMotorVoltage().getValueAsDouble();
        inputs.lowerTunnelTemperature = lowerTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.lowerTunnelCurrentDraw = lowerTunnelMotor.getSupplyCurrent().getValueAsDouble();

        inputs.currentTopRollerVelocity = topRollerMotor.getVelocity().getValueAsDouble();
        inputs.currentTopRollerAppliedVoltage = topRollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.topRollerTemperature = topRollerMotor.getDeviceTemp().getValueAsDouble();
        inputs.topRollerCurrentDraw = topRollerMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void setTargetVoltage(double tunnelVoltage, double hopperVoltage, double rollerVoltage) {
        hopperMotor.setControl(voltageRequest.withOutput(hopperVoltage));
        upperTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
        lowerTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
        topRollerMotor.setControl(voltageRequest.withOutput(rollerVoltage));
    }

    @Override
    public void stop() {
        hopperMotor.setControl(stopRequest);
        upperTunnelMotor.setControl(stopRequest);
        lowerTunnelMotor.setControl(stopRequest);
        topRollerMotor.setControl(stopRequest);
    }
}