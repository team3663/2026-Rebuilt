package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.reusablecode.ReusableCommands;

public class C2026IntakeIO implements IntakeIO {
    /**
     * Minimum pivot angle and maximum pivot angle
     */
    private static final Intake.Constants CONSTANTS = new Intake.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(169.0)
    );

    private final TalonFX intakeMotor;
    private final TalonFX pivotMotor;

    private final double PIVOT_GEAR_RATIO = 30.1587;

    // Motor Requests
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
    private final ReusableCommands reusableCommands = new ReusableCommands();

    public C2026IntakeIO(TalonFX intakeMotor, TalonFX pivotMotor) {
        this.intakeMotor = intakeMotor;
        this.pivotMotor = pivotMotor;

        // Intake Motor Configurations
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        reusableCommands.configurations(intakeConfig, InvertedValue.CounterClockwise_Positive, NeutralModeValue.Coast,60.0, true);
        intakeMotor.getConfigurator().apply(intakeConfig);

        // Pivot Motor Configurations
        TalonFXConfiguration pivotMotor1Config = new TalonFXConfiguration();
        reusableCommands.configurationsWithSensorToMechanism(pivotMotor1Config, InvertedValue.Clockwise_Positive, NeutralModeValue.Brake, PIVOT_GEAR_RATIO, 45.0,true);
        reusableCommands.configurationsForPositionBased(pivotMotor1Config,10.0,3,0,100,0,0,0,3.619,0,0,0.2,-0.2);

        pivotMotor.getConfigurator().apply(pivotMotor1Config);
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        // Pivot Motors
        inputs.currentPivot1Velocity = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.currentPivot1AppliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivot1MotorTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivot1CurrentDraw = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivot1Position = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());

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
        pivotMotor.setControl(stopRequest);
    }

    @Override
    public void resetPivotPosition(double position) {
        pivotMotor.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setTargetPivotPosition(double position) {
        pivotMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetPivotVoltage(double voltage) {
        reusableCommands.setTargetVoltage(pivotMotor, voltage);
    }

    // Intake
    @Override
    public void stopIntake() {
        intakeMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetIntakeVoltage(double voltage) {
        reusableCommands.setTargetVoltage(intakeMotor, voltage);
    }

}
