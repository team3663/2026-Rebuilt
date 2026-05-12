package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2026IntakeRollerIO implements IntakeRollerIO {
    private static final IntakeRoller.Constants CONSTANTS = new IntakeRoller.Constants(
            9.0, 5.0, -5.0
    );

    private final TalonFX rollerMotor;

    // Motor Requests
    private final NeutralOut stopRequest = new NeutralOut();
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);

    public C2026IntakeRollerIO(TalonFX rollerMotor) {
        this.rollerMotor = rollerMotor;

        // Intake Motor Configurations
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 60;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        rollerMotor.getConfigurator().apply(intakeConfig);
    }

    public void updateInputs(IntakeRollerInputs inputs) {
        // Intake Motor
        inputs.currentIntakeVelocity = Units.rotationsToRadians(rollerMotor.getVelocity().getValueAsDouble());
        inputs.currentIntakeAppliedVoltage = rollerMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorTemperature = rollerMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeCurrentDraw = rollerMotor.getSupplyCurrent().getValueAsDouble();

    }

    @Override
    public void stop() {
        rollerMotor.setControl(stopRequest);
    }

    @Override
    public void setTargetVoltage(double voltage) {
        rollerMotor.setControl(voltageRequest.withOutput(voltage));
    }

    public IntakeRoller.Constants getConstants() {
        return CONSTANTS;
    }


}
