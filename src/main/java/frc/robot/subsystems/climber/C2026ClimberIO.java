package frc.robot.subsystems.climber;


import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.Shooter;


public class C2026ClimberIO implements ClimberIO {

    private static final Climber.Constants constants = new Climber.Constants(
            Units.inchesToMeters(9.0), 0.0, Units.inchesToMeters(5.0));
    private static final double CLIMBER_GEAR_RATIO = 33.333336;
    private final TalonFX climbMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);


    public C2026ClimberIO(TalonFX climbMotor) {
        this.climbMotor = climbMotor;


        TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
        climbMotorConfig.Feedback.SensorToMechanismRatio = CLIMBER_GEAR_RATIO;
        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbMotorConfig.MotionMagic.MotionMagicAcceleration = 1.0;
        climbMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0;
        //not sure if the PID values are needed or not
        climbMotorConfig.Slot0.kP = 0.0;
        climbMotorConfig.Slot0.kI = 0.0;
        climbMotorConfig.Slot0.kD = 0.0;


        climbMotor.getConfigurator().apply(climbMotorConfig);
    }


    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.currentClimbPosition = climbMotor.getPosition().getValueAsDouble();
        inputs.currentClimbVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentMotorDraw = climbMotor.getSupplyCurrent().getValueAsDouble();
        inputs.currentMotorTemperature = climbMotor.getDeviceTemp().getValueAsDouble();
        inputs.currentClimbVelocity = climbMotor.getVelocity().getValueAsDouble();
    }


    @Override
    public void stop() {
        setTargetVoltage(0.0);
    }


    @Override
    public void setTargetVoltage(double voltage) {
        climbMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetPosition(double position) {climbMotor.setControl(positionRequest.withPosition(position));}
}
