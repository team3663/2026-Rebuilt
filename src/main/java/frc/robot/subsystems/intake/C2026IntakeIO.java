package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class C2026IntakeIO implements IntakePivotIO {
    private static final IntakePivot.Constants CONSTANTS = new IntakePivot.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)
    );

    private final TalonFX pivotMotor;

    private final NeutralOut stopRequest = new NeutralOut();

    private final double PIVOT_GEAR_RATIO = 30.1587;

    public C2026IntakeIO(TalonFX pivotMotor){
        this.pivotMotor = pivotMotor;

        // Pivot Motor Configurations
        TalonFXConfiguration pivotMotor1Config = new TalonFXConfiguration();
        pivotMotor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        pivotMotor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        pivotMotor1Config.CurrentLimits.SupplyCurrentLimit = 45;
        pivotMotor1Config.CurrentLimits.SupplyCurrentLimitEnable = true;

        pivotMotor1Config.MotionMagic.MotionMagicAcceleration = 10.0;
        pivotMotor1Config.MotionMagic.MotionMagicCruiseVelocity = 3.0;
        pivotMotor1Config.Slot0.kP = 100.0;
        pivotMotor1Config.Slot0.kV = 3.619;
        pivotMotor1Config.MotorOutput.PeakForwardDutyCycle = 0.2;
        pivotMotor1Config.MotorOutput.PeakReverseDutyCycle = -0.2;

        pivotMotor1Config.Feedback.SensorToMechanismRatio = PIVOT_GEAR_RATIO;
        pivotMotor.getConfigurator().apply(pivotMotor1Config);
    }

    public void updateInputs(IntakePivotInputs inputs){
        inputs.currentPivotVelocity = Units.rotationsToRadians(pivotMotor.getVelocity().getValueAsDouble());
        inputs.currentPivotAppliedVoltage = pivotMotor.getMotorVoltage().getValueAsDouble();
        inputs.pivotMotorTemperature = pivotMotor.getDeviceTemp().getValueAsDouble();
        inputs.pivotCurrentDraw = pivotMotor.getSupplyCurrent().getValueAsDouble();
        inputs.currentPivotPosition = Units.rotationsToRadians(pivotMotor.getPosition().getValueAsDouble());
    }

    public static IntakePivot.Constants getConstants() {
        return CONSTANTS;
    }

    public void stopPivot(){
        pivotMotor.setControl(stopRequest);
    }



}
