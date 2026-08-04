package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
    private final FeederIO io;
    private final Constants constants;
    private final FeederInputsAutoLogged inputs = new FeederInputsAutoLogged();

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    public Feeder(FeederIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        FEED,
        DEFAULT,
        OFF
    }

    public enum SystemState {
        FEED,
        DEFAULT,
        OFF
    }

    public void periodic() {
        io.updateInputs(inputs);

        systemState = handleStateTransition();

        // Logging
        Logger.processInputs("Feeder/Inputs", inputs);
        Logger.recordOutput("Feeder/WantedState", wantedState);
        Logger.recordOutput("Feeder/SystemState", systemState);

        applyState();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case FEED:
                yield SystemState.FEED;
            default:
                yield SystemState.OFF;
        };
    }

    private void applyState() {
        double feederVoltage = 0.0;

        switch (systemState) {
            case FEED:
                feederVoltage = constants.feedingVoltage;
                break;
            case OFF:
            case DEFAULT:
                feederVoltage = 0.0;
                break;
        }

        io.setTargetVoltage(feederVoltage);

    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    /**
     * @param feedingVoltage  - Voltage to run the feeder at when feeding
     * @param ejectingVoltage - Voltage to run the feeder at when ejecting
     */
    public record Constants(double feedingVoltage, double ejectingVoltage) {
    }
}
