package frc.robot.subsystems.feeder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class Feeder extends SubsystemBase {

    private final static double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1.0);
    private final FeederIO io;
    private final FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();


    public Feeder(FeederIO io) {
        this.io = io;
    }

    public double targetVoltage = 0.0;
    public double targetVelocity = 0.0;


    public double getCurrentVelocity() {
        return inputs.currentVelocity;
    }

    public double getCurrentVoltage() {
        return inputs.currentAppliedVoltage;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Feeder/targetVoltage", targetVoltage);
        Logger.recordOutput("Feeder/targetVelocity", targetVelocity);
        Logger.processInputs("Feeder/inputs", inputs);
    }

    public Command withVoltage(double voltage) {
        return runEnd(() -> {
                    targetVoltage = voltage;
                    io.setTargetVoltage(voltage);
                },
                io::stop
        );
    }

    public Command withVelocity(double velocity) {
        return runEnd(() -> {
                    targetVelocity = velocity;
                    io.setTargetVelocity(velocity);
                },
                io::stop
        );
    }

    public Command stop() {
        return runOnce(io::stop);
    }

    public boolean isAtTargetVelocity() {
        return isAtFeederVelocity(targetVelocity, VELOCITY_THRESHOLD);
    }

    private boolean isAtFeederVelocity(double currentVelocity, double velocityThreshold) {
        return Math.abs(inputs.currentVelocity - currentVelocity) < velocityThreshold;
    }

}
