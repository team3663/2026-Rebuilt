package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class FeederInputs {
    public double currentVelocity;
    public double currentAppliedVoltage;

    public double motorTemperature;
    public double motorCurrentDraw;

    public boolean canrangeObjectDetected;
    public double canrangeSignalConfidence;
}
