package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

@AutoLog
public class HopperInputs {

    // Inputs for the Hopper Motor
    public double currentHopperVelocity;
    public double currentHopperAppliedVoltage;
    public double hopperTemperature;
    public double hopperCurrentDraw;

    // Inputs for the Upper Tunnel Motor
    public double currentUpperTunnelVelocity;
    public double currentUpperTunnelAppliedVoltage;
    public double upperTunnelTemperature;
    public double upperTunnelCurrentDraw;

    // Inputs for the Lower Tunnel Motor
    public double currentLowerTunnelVelocity;
    public double currentLowerTunnelAppliedVoltage;
    public double lowerTunnelTemperature;
    public double lowerTunnelCurrentDraw;

    // Inputs for the Top Roller Motor
    public double currentTopRollerVelocity;
    public double currentTopRollerAppliedVoltage;
    public double topRollerTemperature;
    public double topRollerCurrentDraw;
}