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

    private final DoubleCircularBuffer rollerCurrentDrawBuffer = new DoubleCircularBuffer(35);

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        rollerCurrentDrawBuffer.addFirst(inputs.topRollerCurrentDraw);
        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/TargetVoltage", targetHopperVoltage);
        Logger.recordOutput("Hopper/TunnelTargetVoltage", targetTunnelVoltage);
        Logger.recordOutput("Hopper/RollerTargetVoltage", targetRollerVoltage);
        Logger.recordOutput("Hopper/AverageRollerCurrentDraw", getAverageRollerCurrentDraw());
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
        }, () -> {
            targetHopperVoltage = 0.0;
            targetRollerVoltage = 0.0;
            targetTunnelVoltage = 0.0;
            io.stop();
        });
    }

    public Command stop() {
        return runOnce(() -> {
            targetHopperVoltage = 0.0;
            targetTunnelVoltage = 0.0;
            targetRollerVoltage = 0.0;
            io.stop();
        });
    }

    public double getAverageRollerCurrentDraw() {
        double sumOfNumbers = 0;

        for (int i = 0; i < rollerCurrentDrawBuffer.size(); i++) {
            sumOfNumbers += rollerCurrentDrawBuffer.get(i);
        }
        return sumOfNumbers / rollerCurrentDrawBuffer.size();
    }

    public void clearTopRollerAverageCurrentDraw() {
        rollerCurrentDrawBuffer.clear();
    }
}