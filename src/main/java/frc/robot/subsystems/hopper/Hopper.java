package frc.robot.subsystems.hopper;

import edu.wpi.first.util.DoubleCircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();
    private double targetHopperVoltage;
    private double targetTunnelVoltage;
    private double targetRollerVoltage;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        currentDraw.addFirst(inputs.topRollerCurrentDraw);
        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/TargetVoltage", targetHopperVoltage);
        Logger.recordOutput("Hopper/TunnelTargetVoltage", targetTunnelVoltage);
        Logger.recordOutput("Hopper/RollerTargetVoltage", targetRollerVoltage);
    }

    public double getTargetHopperVoltage() {
        return targetHopperVoltage;
    }

    public Command withVoltage(double tunnelVoltage, double hopperVoltage, double rollerVoltage) {
        return runEnd(() -> {
            targetHopperVoltage = hopperVoltage;
            targetTunnelVoltage = tunnelVoltage;
            targetRollerVoltage = rollerVoltage;
            io.setTargetVoltage(tunnelVoltage, hopperVoltage, rollerVoltage);
        }, io::stop);
    }

    public Command stop() {
        return runOnce(() -> {
            targetHopperVoltage = 0.0;
            targetTunnelVoltage = 0.0;
            io.stop();
        });
    }

    DoubleCircularBuffer currentDraw = new DoubleCircularBuffer(35);

    public double getAverageTopRollerCurrentDraw() {
        var sumOfNumbers = 0;

        for (int i = 0; i < currentDraw.size(); i++) {
            sumOfNumbers += currentDraw.get(i);
        }
        var average = sumOfNumbers / currentDraw.size();
        Logger.recordOutput("Hopper/currentDrawAverage", average);

        return average;
    }
}