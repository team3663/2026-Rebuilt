package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class IntakeInputs {
    // Pivot Motors
    public double currentPivot1Velocity;
    public double currentPivot1AppliedVoltage;
    public double currentPivot1Position;
    public double pivot1MotorTemperature;
    public double pivot1CurrentDraw;

    public double currentPivot2Velocity;
    public double currentPivot2AppliedVoltage;
    public double currentPivot2Position;
    public double pivot2MotorTemperature;
    public double pivot2CurrentDraw;

    // Intake Motor
    public double currentIntakeVelocity;
    public double currentIntakeAppliedVoltage;
    public double intakeMotorTemperature;
    public double intakeCurrentDraw;
}
