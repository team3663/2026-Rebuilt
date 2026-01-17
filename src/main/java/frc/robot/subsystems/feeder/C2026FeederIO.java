package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class C2026FeederIO implements FeederIO {
    //two motors created, may only need one though
    private final TalonFX motor1;
    private final TalonFX motor2;
    private final CANrange canrange;

    public C2026FeederIO(TalonFX motor1, TalonFX motor2, CANrange canrange){
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.canrange = canrange;

        TalonFXConfiguration motor1Config = new TalonFXConfiguration();
        motor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor1.getConfigurator().apply(motor1Config);
        motor2.getConfigurator().apply(motor1Config);

        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void stop() {
        FeederIO.super.stop();
    }

    @Override
    public void setTargetVoltage(double voltage) {
        FeederIO.super.setTargetVoltage(voltage);
    }

    @Override
    public void setTargetVelocity(double velocity) {
        FeederIO.super.setTargetVelocity(velocity);
    }
}
