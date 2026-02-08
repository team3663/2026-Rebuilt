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
    /**
     * Minimum pivot angle and maximum pivot angle
     */
    private static final Intake.Constants CONSTANTS = new Intake.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(160.0)
    );

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor1;

    private final double PIVOT_GEAR_RATIO = 30.1587;

    // Motor Requests
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2026IntakeIO(TalonFX intakeMotor, TalonFX pivotMotor1, TalonFX pivotMotor2) {
        this.intakeMotor = intakeMotor;
        this.pivotMotor1 = pivotMotor1;

        // Intake Motor Configurations
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(intakeConfig);

        // Pivot Motor Configurations
        TalonFXConfiguration pivotMotor1Config = new TalonFXConfiguration();
        pivotMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        pivotMotor1Config.MotionMagic.MotionMagicAcceleration = 10.0;
        pivotMotor1Config.MotionMagic.MotionMagicCruiseVelocity = 3.0;
        pivotMotor1Config.Slot0.kP = 100.0;
        pivotMotor1Config.Slot0.kV = 3.619;
        pivotMotor1Config.MotorOutput.PeakForwardDutyCycle = 0.2;
        pivotMotor1Config.MotorOutput.PeakReverseDutyCycle = -0.2;

        pivotMotor1Config.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotMotor1.getConfigurator().apply(pivotMotor1Config);

        // Setting one of the pivot motors to follow the other motor
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // Pivot Motors
        inputs.currentPivot1Velocity = Units.rotationsToRadians(pivotMotor1.getVelocity().getValueAsDouble());
        inputs.currentPivot1AppliedVoltage = pivotMotor1.getMotorVoltage().getValueAsDouble();
        inputs.pivot1MotorTemperature = pivotMotor1.getDeviceTemp().getValueAsDouble();
        inputs.pivot1CurrentDraw = pivotMotor1.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivot1Position = Units.rotationsToRadians(pivotMotor1.getPosition().getValueAsDouble());

//        inputs.currentPivot2Velocity = Units.rotationsToRadians(pivotMotor2.getVelocity().getValueAsDouble());
//        inputs.currentPivot2AppliedVoltage = pivotMotor2.getMotorVoltage().getValueAsDouble();
//        inputs.pivot2MotorTemperature = pivotMotor2.getDeviceTemp().getValueAsDouble();
//        inputs.pivot2CurrentDraw = pivotMotor2.getSupplyCurrent().getValueAsDouble();
//        inputs.currentPivot2Position = Units.rotationsToRadians(pivotMotor2.getPosition().getValueAsDouble());

        // Intake Motor
        inputs.currentIntakeVelocity = Units.rotationsToRadians(intakeMotor.getVelocity().getValueAsDouble());
        inputs.currentIntakeAppliedVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorTemperature = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeCurrentDraw = intakeMotor.getSupplyCurrent().getValueAsDouble();

    }

    @Override
    public Intake.Constants getConstants() {
        return CONSTANTS;
    }

    // Pivot
    @Override
    public void stopPivot() {
        pivotMotor1.setControl(stopRequest);
    }

    @Override
    public void resetPivotPosition(double position) {
        pivotMotor1.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setTargetPivotPosition(double position) {
        pivotMotor1.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetPivotVoltage(double voltage) {
        pivotMotor1.setControl(voltageRequest.withOutput(voltage));
    }

    // Intake
    @Override
    public void stopIntake() {
        intakeMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetIntakeVoltage(double voltage) {
        intakeMotor.setControl(voltageRequest.withOutput(voltage));
    }

}
