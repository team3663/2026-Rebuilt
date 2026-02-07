package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
    //upper hooks motor
    public double currentUpperHooksMotorAppliedVoltage;
    public double currentUpperHooksMotorAppliedCurrent;
    public double currentUpperHooksMotorPosition;
    public double currentUpperHooksMotorVelocity;
    public double upperHooksMotorTempature;

    //lower hooks motor
    public double currentLowerHooksMotorAppliedVoltage;
    public double currentLowerHooksMotorAppliedCurrent;
    public double currentLowerHooksMotorPosition;
    public double currentLowerHooksMotorVelocity;
    public double lowerHooksMotorTempature;

    //climb motor 1
    public double currentClimbMotor1AppliedVoltage;
    public double currentClimbMotor1AppliedCurrent;
    public double currentClimbMotor1Position;
    public double currentClimbMotor1Velocity;
    public double climbMotor1Tempature;

    //climb motor 2
    public double currentClimbMotor2AppliedVoltage;
    public double currentClimbMotor2AppliedCurrent;
    public double currentClimbMotor2Position;
    public double currentClimbMotor2Velocity;
    public double climbMotor2Tempature;
}