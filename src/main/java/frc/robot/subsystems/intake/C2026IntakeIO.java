package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2026IntakeIO implements IntakeIO {
    // TODO get minimum and maximum angles for pivot
    // TODO finish pivot
    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;
    private final TalonFX pivotMotor2;

    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2026IntakeIO(TalonFX intakeMotor, TalonFX pivotMotor1, TalonFX pivotMotor2){
        this.intakeMotor = intakeMotor;
        this.pivotMotor1 = pivotMotor1;
        this.pivotMotor2 = pivotMotor2;

        // Intake Motor Configurations
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(intakeConfig);

        // Pivot Motor Configurations
        TalonFXConfiguration pivotMotor1Config = new TalonFXConfiguration();
//        motor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor1.getConfigurator().apply(pivotMotor1Config);

        TalonFXConfiguration pivotMotor2Config = new TalonFXConfiguration();
//        pivotMotor2Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor2Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor2.getConfigurator().apply(pivotMotor2Config);

        pivotMotor2.setControl(new Follower(pivotMotor1.getDeviceID(), MotorAlignmentValue.Aligned));
    }

    @Override
    public void updateInputs(IntakeInputs inputs){
        // Pivot Motors
        inputs.currentPivot1Velocity = Units.rotationsToRadians(pivotMotor1.getVelocity().getValueAsDouble());
        inputs.currentPivot1AppliedVoltage = pivotMotor1.getMotorVoltage().getValueAsDouble();
        inputs.pivot1MotorTemperature = pivotMotor1.getDeviceTemp().getValueAsDouble();
        inputs.pivot1CurrentDraw = pivotMotor1.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivot1Position = Units.rotationsToRadians(pivotMotor1.getPosition().getValueAsDouble());

        inputs.currentPivot2Velocity = Units.rotationsToRadians(pivotMotor2.getVelocity().getValueAsDouble());
        inputs.currentPivot2AppliedVoltage = pivotMotor2.getMotorVoltage().getValueAsDouble();
        inputs.pivot2MotorTemperature = pivotMotor2.getDeviceTemp().getValueAsDouble();
        inputs.pivot2CurrentDraw = pivotMotor2.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivot2Position = Units.rotationsToRadians(pivotMotor2.getPosition().getValueAsDouble());

        // Intake Motor
        inputs.currentIntakeVelocity = Units.rotationsToRadians(intakeMotor.getVelocity().getValueAsDouble());
        inputs.currentIntakeAppliedVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorTemperature = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeCurrentDraw = intakeMotor.getSupplyCurrent().getValueAsDouble();

    }

    @Override
    public void stopIntake(){
        intakeMotor.setControl(stopRequest);
    }

    @Override
    public void stopPivot(){
        pivotMotor1.setControl(stopRequest);
    }

    @Override
    public void resetPivotPosition(double position){
        pivotMotor1.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setTargetPosition(double position){
        pivotMotor1.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetVoltagePivot(double voltage){
        pivotMotor1.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetVoltageIntake(double voltage){
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

}
