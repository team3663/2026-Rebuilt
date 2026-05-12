package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2026IntakePivotIO implements IntakePivotIO {
    /**
     * Minimum pivot angle and maximum pivot angle
     */
    private static final IntakePivot.Constants CONSTANTS = new IntakePivot.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(169.0), Units.degreesToRadians(2.0),
            Units.degreesToRadians(163.0), Units.degreesToRadians(125.0), Units.degreesToRadians(163.0),
            Units.degreesToRadians(48.339844)
    );

    private final TalonFX pivotMotor;

    private final double PIVOT_GEAR_RATIO = 30.1587;

    // Motor Requests
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2026IntakePivotIO(TalonFX pivotMotor) {
        this.pivotMotor = pivotMotor;

        // Pivot Motor Configurations
        TalonFXConfiguration pivotMotorConfig = new TalonFXConfiguration();
        pivotMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimit = 45;
        pivotMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotorConfig.MotionMagic.MotionMagicAcceleration = 10.0;
        pivotMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 3.0;
        pivotMotorConfig.Slot0.kP = 100.0;
        pivotMotorConfig.Slot0.kV = 3.619;
        pivotMotorConfig.MotorOutput.PeakForwardDutyCycle = 0.2;
        pivotMotorConfig.MotorOutput.PeakReverseDutyCycle = -0.2;

        pivotMotorConfig.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotMotor.getConfigurator().apply(pivotMotorConfig);
    }

    @Override
    public void updateInputs(IntakePivotInputs inputs) {
        // Pivot Motors
        inputs.currentPivotVelocity = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.currentPivotAppliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotCurrentDraw = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivotPosition = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());
    }

    public IntakePivot.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void stop() {
        pivotMotor.setControl(stopRequest);
    }

    @Override
    public void resetPosition(double position) {
        pivotMotor.setPosition(Units.radiansToRotations(position));
    }

    @Override
    public void setTargetPosition(double position) {
        pivotMotor.setControl(positionRequest.withPosition(Units.radiansToRotations(position)));
    }

    @Override
    public void setTargetVoltage(double voltage) {
        pivotMotor.setControl(voltageRequest.withOutput(voltage));
    }
}
