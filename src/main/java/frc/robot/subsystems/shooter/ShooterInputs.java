package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class ShooterInputs {
    // Hood Motor
    public double currentHoodAppliedVoltage;
    public double currentHoodVelocity;
    public double currentHoodPosition;

    public double hoodMotorTemperature;
    public double currentHoodDraw;

    // Turret Motor
    public double currentTurretAppliedVoltage;
    public double currentTurretVelocity;
    public double currentTurretPosition;

    public double currentTurretEncoderPosition1;
    public double currentTurretEncoderPosition2;

    public double turretMotorTemperature;
    public double currentTurretDraw;

    // Shooter Motor 1
    public double currentShooterAppliedVoltage1;
    public double currentShooterVelocity1;

    public double shooterMotorTemperature1;
    public double currentShooterDraw1;

    // Shooter Motor 2
    public double currentShooterAppliedVoltage2;
    public double currentShooterVelocity2;

    public double shooterMotorTemperature2;
    public double currentShooterDraw2;
}
