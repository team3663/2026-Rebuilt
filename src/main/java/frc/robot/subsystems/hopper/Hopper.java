package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
    private final HopperIO io;
    private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();
    private double targetVoltage = 0.0;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper/Inputs", inputs);
        Logger.recordOutput("Hopper/TargetVoltage", targetVoltage);
    }

    public double getTargetVoltage() {
        return targetVoltage;
    }

    public Command withVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltage(voltage);
        }, io::stop);
    }

    public Command stop() {
        return runOnce(() -> {
            targetVoltage = 0.0;
            io.stop();
        });
    }
}