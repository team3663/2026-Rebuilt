package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class C2026ClimberIO implements ClimberIO {
    private static final Climber.Constants CONSTANTS = new Climber.Constants(Units.inchesToMeters(68.0), Units.degreesToRadians(180.0), Units.degreesToRadians(180.0));

    private static final double DEPLOY_GEAR_RATIO = 1.0;
    private static final double CLIMB_GEAR_RATIO = 1.0;

    private static final double LOWER_HOOKS_DEPLOY_ANGLE = Units.degreesToRadians(90.0);
    private static final double UPPER_HOOKS_DEPLOY_ANGLE = Units.degreesToRadians(90.0);

    private final TalonFX upperHooksMotor;
    private final TalonFX lowerHooksMotor;
    private final TalonFX climbMotor1;
    private final TalonFX climbMotor2;

    private final PositionVoltage positionRequest = new PositionVoltage(0.0);
    private final VoltageOut voltageRequest = new VoltageOut(0.0);
    private final NeutralOut stopRequest = new NeutralOut();

    public C2026ClimberIO(TalonFX upperHooksMotor, TalonFX climbMotor1, TalonFX climbMotor2, TalonFX lowerHooksMotor) {
        this.upperHooksMotor = upperHooksMotor;
        this.lowerHooksMotor = lowerHooksMotor;
        this.climbMotor1 = climbMotor1;
        this.climbMotor2 = climbMotor2;

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

        upperHooksMotor.getConfigurator().apply(deployConfig);
        lowerHooksMotor.getConfigurator().apply(deployConfig);

        //climb motor(s) config
        TalonFXConfiguration climbConfig = new TalonFXConfiguration();
        climbConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        climbConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        climbConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
        climbConfig.MotionMagic.MotionMagicAcceleration = 1;

        climbConfig.Slot0.kV = 0.0;
        climbConfig.Slot0.kA = 0.0;
        climbConfig.Slot0.kP = 0.1;
        climbConfig.Slot0.kI = 0.0;
        climbConfig.Slot0.kD = 0.0;
        climbConfig.Slot0.kG = 0.0;
        climbConfig.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        climbMotor1.getConfigurator().apply(climbConfig);
        climbMotor2.getConfigurator().apply(climbConfig);

        climbMotor2.setControl(new Follower(climbMotor1.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    @Override
    public void updateInputs(ClimberInputs inputs) {
        //deploy upper hooks motor
        inputs.currentUpperHooksMotorAppliedVoltage = upperHooksMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentUpperHooksMotorAppliedCurrent = upperHooksMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentUpperHooksMotorPosition = Units.radiansToDegrees(upperHooksMotor.getPosition().getValueAsDouble());
        inputs.currentUpperHooksMotorVelocity = upperHooksMotor.getVelocity().getValueAsDouble();
        inputs.upperHooksMotorTempature = upperHooksMotor.getDeviceTemp().getValueAsDouble();

        //deploy lower hooks motor
        inputs.currentLowerHooksMotorAppliedVoltage = lowerHooksMotor.getMotorVoltage().getValueAsDouble();
        inputs.currentLowerHooksMotorAppliedCurrent = lowerHooksMotor.getStatorCurrent().getValueAsDouble();
        inputs.currentLowerHooksMotorPosition = Units.radiansToDegrees(lowerHooksMotor.getPosition().getValueAsDouble());
        inputs.currentLowerHooksMotorVelocity = lowerHooksMotor.getVelocity().getValueAsDouble();
        inputs.lowerHooksMotorTempature = lowerHooksMotor.getDeviceTemp().getValueAsDouble();

        //climb motor 1
        inputs.currentClimbMotor1AppliedVoltage = climbMotor1.getMotorVoltage().getValueAsDouble();
        inputs.currentClimbMotor1AppliedCurrent = climbMotor1.getStatorCurrent().getValueAsDouble();
        inputs.currentClimbMotor1Position = Units.radiansToDegrees(climbMotor1.getPosition().getValueAsDouble());
        inputs.currentClimbMotor1Velocity = climbMotor1.getVelocity().getValueAsDouble();
        inputs.climbMotor1Tempature = climbMotor1.getDeviceTemp().getValueAsDouble();

        //climb motor 2
        inputs.currentClimbMotor2AppliedVoltage = climbMotor2.getMotorVoltage().getValueAsDouble();
        inputs.currentClimbMotor2AppliedCurrent = climbMotor2.getStatorCurrent().getValueAsDouble();
        inputs.currentClimbMotor2Position = Units.radiansToDegrees(climbMotor2.getPosition().getValueAsDouble());
        inputs.currentClimbMotor2Velocity = climbMotor2.getVelocity().getValueAsDouble();
        inputs.climbMotor2Tempature = climbMotor2.getDeviceTemp().getValueAsDouble();
    }

    @Override
    public Climber.Constants getConstants() {
        return CONSTANTS;
    }

    /**VOLTAGE**/
    @Override
    public void setTargetUpperHooksVoltage(double voltage) {
        upperHooksMotor.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void setTargetLowerHooksVoltage(double voltage) {
        lowerHooksMotor.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void setTargetClimbVoltage(double voltage) {
        climbMotor1.setControl(voltageRequest.withOutput(voltage));
        climbMotor2.setControl(voltageRequest.withOutput(voltage));
    }
    @Override
    public void retractHooks(){
        upperHooksMotor.setControl(voltageRequest.withOutput(-1.0));
    }

    /**POSITION**/
    @Override
    public void goToPosition(double position) {
        climbMotor1.setControl(positionRequest.withPosition(position));
        climbMotor2.setControl(positionRequest.withPosition(position));
    }
    @Override
    public void setTargetUpperHooksPosition(double degrees) {
        upperHooksMotor.setControl(positionRequest.withPosition(degrees));
    }
    @Override
    public void setTargetLowerHooksPosition(double degrees) {
        lowerHooksMotor.setControl(positionRequest.withPosition(degrees));
    }
    @Override
    public void deployHooks(){
        lowerHooksMotor.setControl(positionRequest.withPosition(LOWER_HOOKS_DEPLOY_ANGLE));
        upperHooksMotor.setControl(positionRequest.withPosition(UPPER_HOOKS_DEPLOY_ANGLE));
    }
    @Override
    public void resetHooksPosition(double upperHooksPosition, double lowerHooksPosition) {
        upperHooksMotor.setPosition(upperHooksPosition);
        lowerHooksMotor.setPosition(lowerHooksPosition);
    }
    @Override
    public void resetClimbPosition(double position) {
        climbMotor1.setPosition(position);
    }


    /**stoppers**/
    @Override
    public void stopUpperHooks() {
        upperHooksMotor.setControl(stopRequest);
    }
    @Override
    public void stopLowerHooks() {
        lowerHooksMotor.setControl(stopRequest);
    }
    @Override
    public void stopClimb() {
        climbMotor1.setControl(stopRequest);
        climbMotor2.setControl(stopRequest);
    }


    public void stopClimber() {
        this.stopUpperHooks();
        this.stopLowerHooks();
        stopClimb();
    }
}