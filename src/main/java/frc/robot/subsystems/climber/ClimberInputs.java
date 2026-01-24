package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ClimberInputs {
    //deploy motor
    public double currentDeployMotorAppliedVoltage;
    public double currentDeployMotorAppliedCurrent;
    public double currentDeployMotorPosition;
    public double currentDeployMotorVelocity;
    public double deployMotorTempature;

    //climb motor
    public double currentClimbMotorAppliedVoltage;
    public double currentClimbMotorAppliedCurrent;
    public double currentClimbMotorPosition;
    public double currentClimbMotorVelocity;
    public double climbMotorTempature;
}