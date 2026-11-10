package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.subsystems.reusablecode.ReusableCommands;

public class C2026HopperIO implements HopperIO {
    private static final double TUNNEL_GEAR_RATIO = 1.0 / 2.0;

    private final TalonFX hopperMotor;
    private final TalonFX upperTunnelMotor;
    private final TalonFX lowerTunnelMotor;
    private final TalonFX topRollerMotor;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final ReusableCommands reusableCommands = new ReusableCommands();

    public C2026HopperIO(TalonFX hopperMotor, TalonFX upperTunnelMotor, TalonFX lowerTunnelMotor, TalonFX topRollerMotor) {
        this.hopperMotor = hopperMotor;
        this.upperTunnelMotor = upperTunnelMotor;
        this.lowerTunnelMotor = lowerTunnelMotor;
        this.topRollerMotor = topRollerMotor;

        TalonFXConfiguration config = new TalonFXConfiguration();
        reusableCommands.configurations(config, InvertedValue.Clockwise_Positive, NeutralModeValue.Coast, 20.0, true);

        TalonFXConfiguration upperTunnelConfig = new TalonFXConfiguration();
        reusableCommands.configurationsWithSensorToMechanism(upperTunnelConfig, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast, TUNNEL_GEAR_RATIO, 30.0, true);

        TalonFXConfiguration lowerTunnelConfig = new TalonFXConfiguration();
        reusableCommands.configurationsWithSensorToMechanism(lowerTunnelConfig, InvertedValue.Clockwise_Positive, NeutralModeValue.Coast, TUNNEL_GEAR_RATIO, 30.0, true);

        TalonFXConfiguration topRollerConfig = new TalonFXConfiguration();
        reusableCommands.configurationsWithSensorToMechanism(topRollerConfig, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast, TUNNEL_GEAR_RATIO, 40.0, true);

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
        reusableCommands.setTargetVoltage(hopperMotor, hopperVoltage);
        reusableCommands.setTargetVoltage(upperTunnelMotor, tunnelVoltage);
        reusableCommands.setTargetVoltage(lowerTunnelMotor, tunnelVoltage);
        reusableCommands.setTargetVoltage(topRollerMotor, rollerVoltage);
    }

    @Override
    public void stop() {
        hopperMotor.setControl(stopRequest);
        upperTunnelMotor.setControl(stopRequest);
        lowerTunnelMotor.setControl(stopRequest);
        topRollerMotor.setControl(stopRequest);
    }
}