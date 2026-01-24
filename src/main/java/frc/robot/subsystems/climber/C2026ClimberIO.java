package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2026ClimberIO implements ClimberIO {
    private static final double DEPLOY_GEAR_RATIO = 1.0;
    private static final double CLIMB_GEAR_RATIO = 1.0;

    private final TalonFX deployMotor;
    private final TalonFX climbMotor;

    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2026ClimberIO(TalonFX deployMotor, TalonFX climbMotor) {
        this.deployMotor = deployMotor;
        this.climbMotor = climbMotor;



        //TODO get config values
        //deploy motor config
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        deployConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        deployConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        deployConfig.MotionMagic.MotionMagicAcceleration = 1;

        deployConfig.Slot0.kV = 0.0;
        deployConfig.Slot0.kA = 0.0;
        deployConfig.Slot0.kP = 0.0;
        deployConfig.Slot0.kI = 0.0;
        deployConfig.Slot0.kD = 0.0;
        deployConfig.Slot0.kG = 0.0;
        deployConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        deployMotor.getConfigurator().apply(deployConfig);

        //climb motor(s) config
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climbConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        climbConfig.MotionMagic.MotionMagicAcceleration = 1;

        climbConfig.Slot0.kV = 0.0;
        climbConfig.Slot0.kA = 0.0;
        climbConfig.Slot0.kP = 0.0;
        climbConfig.Slot0.kI = 0.0;
        climbConfig.Slot0.kD = 0.0;
        climbConfig.Slot0.kG = 0.0;
        climbConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        climbMotor.getConfigurator().apply(climbConfig);
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        //deploy motor
        inputs.currentDeployMotorAppliedVoltage = deployMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentDeployMotorAppliedCurrent = deployMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentDeployMotorPosition = Units.radiansToDegrees(deployMotor.getPosition().getValueAsDouble());
        inputs.currentDeployMotorVelocity = deployMotor.getVelocity().getValueAsDouble();
        inputs.deployMotorTempature = deployMotor.getDeviceTemp().getValueAsDouble();

        //climb motor
        inputs.currentClimbMotorAppliedVoltage = climbMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentClimbMotorAppliedCurrent = climbMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentClimbMotorPosition = Units.radiansToDegrees(climbMotor.getPosition().getValueAsDouble());
        inputs.currentClimbMotorVelocity = climbMotor.getVelocity().getValueAsDouble();
        inputs.climbMotorTempature = climbMotor.getDeviceTemp().getValueAsDouble();
    }

    //deploy motor
    @Override
    public void setTargetDeployVoltage(double voltage) {
        deployMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetDeployPosition(double degrees) {
        deployMotor.setControl(positionRequest.withPosition(Units.degreesToRadians(degrees)));
    }

    @Override
    public void resetDeployPosition(double position) {
        deployMotor.setPosition(Units.degreesToRotations(position));
    }

    //climb motor(s)
    @Override
    public void setTargetClimbVoltage(double voltage) {
        climbMotor.setControl(voltageRequest.withOutput(voltage));
    }

    @Override
    public void setTargetClimbPosition(double degrees) {
        climbMotor.setControl(positionRequest.withPosition(Units.degreesToRadians(degrees)));
    }

    @Override
    public void resetClimbPosition(double position) {
        climbMotor.setPosition(Units.degreesToRotations(position));
    }

    //stoppers
    @Override
    public void stopDeploy() {
        deployMotor.setControl(stopRequest);
    }

    @Override
    public void stopClimb() {
        climbMotor.setControl(stopRequest);
    }

    public void stopClimber() {
        stopDeploy();
        stopClimb();
    }
}