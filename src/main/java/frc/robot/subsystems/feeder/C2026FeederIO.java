package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.util.Units;

public class C2026FeederIO implements FeederIO {
    //PROXIMITY_THRESHOLD is threshold for object detection
    //PROXIMITY_HYSTERESIS is the additional distance beyond the threshold that
    // the object will still register as detected
    //MIN_SIGNAL_STRENGTH is the smallest strength of the signal
    // that will allow the object to be counted
    // TODO get real values on robot
    private final static double PROXIMITY_THRESHOLD = 0.0;
    private final static double PROXIMITY_HYSTERESIS = 0.0;
    private final static double MIN_SIGNAL_STRENGTH = 0.0;
    //two motors created, may only need one though
    //TODO figure out how many motors will be used
    private final TalonFX motor1; //14;
    private final TalonFX motor2; //15;
    private final CANrange canrange;

    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

    public C2026FeederIO(TalonFX motor1, TalonFX motor2, CANrange canrange) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.canrange = canrange;

        TalonFXConfiguration motor1Config = new TalonFXConfiguration();
        motor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //not sure if the PID values are needed or not
//        motor1Config.Slot0.kP = 0.0;
//        motor1Config.Slot0.kI = 0.0;
//        motor1Config.Slot0.kD = 0.0;

        motor1.getConfigurator().apply(motor1Config);
        motor2.getConfigurator().apply(motor1Config);

        motor2.setControl(new Follower(motor1.getDeviceID(), MotorAlignmentValue.Opposed));

        CANrangeConfiguration canRangeConfig = new CANrangeConfiguration();
        canRangeConfig.ProximityParams.ProximityThreshold = Units.inchesToMeters(PROXIMITY_THRESHOLD);
        canRangeConfig.ProximityParams.ProximityHysteresis = Units.inchesToMeters(PROXIMITY_HYSTERESIS);
        canRangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = MIN_SIGNAL_STRENGTH;
        canRangeConfig.ToFParams.withUpdateMode(UpdateModeValue.LongRangeUserFreq);
        canrange.getConfigurator().apply(canRangeConfig);
    }

    @Override
    public void stop() {
        motor1.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        motor1.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetVelocity(double velocity) {
        motor1.setControl(velocityVoltage.withVelocity(velocity));
    }
}