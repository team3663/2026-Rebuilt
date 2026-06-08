package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeRoller;
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
        EJECT,
        DEFAULT,
        OFF
    }

    public enum SystemState {
        FEED,
        EJECT,
        DEFAULT,
        OFF
    }

    public void periodic(){
        io.updateInputs(inputs);

        // Logging
        Logger.processInputs("Feeder/Inputs", inputs);
        Logger.recordOutput("Feeder/WantedState", wantedState);
        Logger.recordOutput("Feeder/SystemState", systemState);
    }

    private SystemState handleStateTransition(){
        return switch (wantedState) {
            case FEED:
                yield SystemState.FEED;
            case EJECT:
                yield SystemState.EJECT;
            default:
                yield SystemState.OFF
        };
    }

    private void applyState() {
        double feederVoltage = 0.0;

        switch (systemState) {
            case FEED:
                feederVoltage = constants.feedingVoltage;
                break;
            case EJECT:
                feederVoltage = constants.ejectingVoltage;
                break;
            case OFF:
            default:
                feederVoltage = 0.0;
                break;
        }

        io.setTargetVoltage(feederVoltage);

    }

    public void setWantedState(WantedState wantedState){
        this.wantedState = wantedState;
    }

    /**
     * @param feedingVoltage - Voltage to run the feeder at when feeding
     * @param ejectingVoltage - Voltage to run the feeder at when ejecting
     */
    public record Constants(double feedingVoltage, double ejectingVoltage) {
    }
}
