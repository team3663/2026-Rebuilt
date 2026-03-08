package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();
    private double targetHopperVoltage;
    private double targetCorneringVoltage;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/TargetVoltage", targetHopperVoltage);
        Logger.recordOutput("Hopper/CorneringTargetVoltage", targetCorneringVoltage);
    }

    public double getTargetHopperVoltage() {
        return targetHopperVoltage;
    }

    public Command withVoltage(double corneringVoltage, double hopperVoltage) {
        return runEnd(() -> {
            targetHopperVoltage = hopperVoltage;
            targetCorneringVoltage = corneringVoltage;
            io.setTargetVoltage(corneringVoltage, hopperVoltage);
        }, io::stop);
    }

    public Command stop() {
        return runOnce(() -> {
            targetHopperVoltage = 0.0;
            io.stop();
        });
    }
}