package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026HopperIO implements HopperIO {
    private static final Hopper.Constants CONSTANTS = new Hopper.Constants(7.0, 8.0, 8.0, -4.0);

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

        // Configuring the hopper motor
        TalonFXConfiguration hopperConfig = new TalonFXConfiguration();
        hopperConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperConfig.CurrentLimits.SupplyCurrentLimit = 20;
        hopperConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Configuring the upper tunnel motor
        TalonFXConfiguration upperTunnelConfig = new TalonFXConfiguration();
        upperTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        upperTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimit = 30;
        upperTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        upperTunnelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // Configuring the lower tunnel motor
        TalonFXConfiguration lowerTunnelConfig = new TalonFXConfiguration();
        lowerTunnelConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        lowerTunnelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimit = 30;
        lowerTunnelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        lowerTunnelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        // Configuring the top roller motor
        TalonFXConfiguration topRollerConfig = new TalonFXConfiguration();
        topRollerConfig.Feedback.SensorToMechanismRatio = TUNNEL_GEAR_RATIO;
        topRollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        topRollerConfig.CurrentLimits.SupplyCurrentLimit = 40;
        topRollerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        topRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        // applying motor configs

        hopperMotor.getConfigurator().apply(hopperConfig);
        upperTunnelMotor.getConfigurator().apply(upperTunnelConfig);
        lowerTunnelMotor.getConfigurator().apply(lowerTunnelConfig);
        topRollerMotor.getConfigurator().apply(topRollerConfig);


        topRollerMotor.getSupplyCurrent()
                .setUpdateFrequency(50.0);
    }

    @Override
    public void updateInputs(HopperInputs inputs) {
        // updating hopper inputs
        inputs.currentHopperVelocity = hopperMotor.getVelocity().getValueAsDouble();
        inputs.currentHopperAppliedVoltage = hopperMotor.getMotorVoltage().getValueAsDouble();
        inputs.hopperTemperature = hopperMotor.getDeviceTemp().getValueAsDouble();
        inputs.hopperCurrentDraw = hopperMotor.getSupplyCurrent().getValueAsDouble();

        // updating upper tunnel inputs
        inputs.currentUpperTunnelVelocity = upperTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentUpperTunnelAppliedVoltage = upperTunnelMotor.getMotorVoltage().getValueAsDouble();
        inputs.upperTunnelTemperature = upperTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.upperTunnelCurrentDraw = upperTunnelMotor.getSupplyCurrent().getValueAsDouble();

        // updating lower tunnel inputs
        inputs.currentLowerTunnelVelocity = lowerTunnelMotor.getVelocity().getValueAsDouble();
        inputs.currentLowerTunnelAppliedVoltage = lowerTunnelMotor.getMotorVoltage().getValueAsDouble();
        inputs.lowerTunnelTemperature = lowerTunnelMotor.getDeviceTemp().getValueAsDouble();
        inputs.lowerTunnelCurrentDraw = lowerTunnelMotor.getSupplyCurrent().getValueAsDouble();

        // updating top roller inputs
        inputs.currentTopRollerVelocity = topRollerMotor.getVelocity().getValueAsDouble();
        inputs.currentTopRollerAppliedVoltage = topRollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.topRollerTemperature = topRollerMotor.getDeviceTemp().getValueAsDouble();
        inputs.topRollerCurrentDraw = topRollerMotor.getSupplyCurrent().getValueAsDouble();
    }

    @Override
    public void stop() {
        hopperMotor.setControl(stopRequest);
        upperTunnelMotor.setControl(stopRequest);
        lowerTunnelMotor.setControl(stopRequest);
        topRollerMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double tunnelVoltage, double hopperVoltage, double rollerVoltage){
        hopperMotor.setControl(voltageRequest.withOutput(hopperVoltage));
        upperTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
        lowerTunnelMotor.setControl(voltageRequest.withOutput(tunnelVoltage));
        topRollerMotor.setControl(voltageRequest.withOutput(rollerVoltage));
    }

    public Hopper.Constants getConstants(){
        return CONSTANTS;
    }

}