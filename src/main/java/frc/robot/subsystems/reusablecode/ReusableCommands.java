package frc.robot.subsystems.reusablecode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ReusableCommands {
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public TalonFXConfiguration configurations(TalonFXConfiguration config,
                                               InvertedValue inverted,
                                               NeutralModeValue neutralModeValue,
                                               double currentLimit,
                                               boolean currentLimitEnabled){
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = neutralModeValue;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = currentLimitEnabled;
        return config;
    }

    public TalonFXConfiguration configurationsWithSensorToMechanism(TalonFXConfiguration config,
                                                                    InvertedValue inverted,
                                                                    NeutralModeValue neutralModeValue,
                                                                    double sensorToMechanismRatio,
                                                                    double currentLimit,
                                                                    boolean currentLimitEnabled){
        config.MotorOutput.Inverted = inverted;
        config.MotorOutput.NeutralMode = neutralModeValue;
        config.Feedback.SensorToMechanismRatio = sensorToMechanismRatio;
        config.CurrentLimits.SupplyCurrentLimit = currentLimit;
        config.CurrentLimits.SupplyCurrentLimitEnable = currentLimitEnabled;
        return config;
    }

    public TalonFXConfiguration configurationsForPositionBased(TalonFXConfiguration config,
                                                               double MotionMagicAccel,
                                                               double MotionMagicCV,
                                                               double MotionMagicJerk,
                                                               double kp,
                                                               double ki,
                                                               double kd,
                                                               double ka,
                                                               double kv,
                                                               double kg,
                                                               double ks,
                                                               double peakForward,
                                                               double peakReverse){
        if(peakReverse > 1.0){
            peakReverse = 1.0;
        }
        if(peakReverse < -1.0){
            peakReverse = -1.0;
        }
        if(peakForward > 1.0){
            peakForward = 1.0;
        }
        if(peakForward < -1.0){
            peakForward = -1.0;
        }

        //all defaults are zero except peak forward/reverse which are 1/-1
        config.MotionMagic.MotionMagicAcceleration = MotionMagicAccel;
        config.MotionMagic.MotionMagicCruiseVelocity = MotionMagicCV;
        config.MotionMagic.MotionMagicJerk = MotionMagicJerk;
        config.Slot0.kP = kp;
        config.Slot0.kI = ki;
        config.Slot0.kD = kd;
        config.Slot0.kA = ka;
        config.Slot0.kG = kg;
        config.Slot0.kV = kv;
        config.Slot0.kS = ks;
        config.MotorOutput.PeakForwardDutyCycle = peakForward;
        config.MotorOutput.PeakReverseDutyCycle = peakReverse;
        return(config);
    }

    public void setTargetVoltage(TalonFX motor, double voltage){
        motor.setControl(voltageRequest.withOutput(voltage));
    }
}
