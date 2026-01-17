package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class Hopper extends SubsystemBase {
    private HopperIO io;
    private HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();
    private double targetVoltage = 0.0;

    public Hopper(HopperIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hopper", inputs);
    }

    public double getTargetVoltage() {return targetVoltage;}

    public Command withVoltage(double voltage) {
        targetVoltage = voltage;
        return run(() -> {
            io.setVoltage(voltage);
        });
    }

    public Command stop() {
        targetVoltage = 0.0;
        return runOnce(() -> {io.stop();});
    }
}