package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FeederInputs {
    public double currentVelocity;
    public double currentAppliedVoltage;

    public double motor1Temperature;
    public double motor1CurrentDraw;

    public boolean canrangeObjectDetected;
    public double canrangeSignalConfidence;
}
