package frc.robot.subsystems.climber;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;


public class C2026ClimberIO implements ClimberIO {


    public TalonFX climbMotor;
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();
    private final PositionVoltage positionRequest = new PositionVoltage(0.0);


    public C2026ClimberIO(TalonFX climbMotor) {
        this.climbMotor = climbMotor;


        TalonFXConfiguration climbMotorConfig = new TalonFXConfiguration();
        climbMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        climbMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        //not sure if the PID values are needed or not
//        climbMotorConfig.Slot0.kP = 0.0;
//        climbMotorConfig.Slot0.kI = 0.0;
//        climbMotorConfig.Slot0.kD = 0.0;


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
        climbMotor.setControl(stopRequest);
    }


    @Override
    public void setTargetVoltage(double voltage) {
        climbMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetPosition(double position) {
        climbMotor.setControl(positionRequest.withPosition(position));
    }
}
