package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakeRoller extends SubsystemBase {
    private final C2026IntakeRollerIO io;
    private final Constants constants;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;


    public IntakeRoller(C2026IntakeRollerIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        INTAKE,
        FEED,
        EJECT,
        DEFAULT,
        OFF
    }

    public enum SystemState {
        INTAKE,
        FEED,
        EJECT,
        DEFAULT,
        OFF
    }

    public void periodic() {
        io.updateInputs(inputs);

        // Logging
        Logger.recordOutput("Intake/Roller/WantedState", wantedState);
        Logger.recordOutput("Intake/Roller/SystemState", systemState);
        Logger.processInputs("Intake/Roller/Inputs", inputs);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case INTAKE: {
                yield SystemState.INTAKE;
            }
            case FEED: {
                yield SystemState.FEED;
            }
            case EJECT: {
                yield SystemState.EJECT;
            }
            default: {
                yield SystemState.OFF;
            }
        };
    }

    private void applyState() {
        double rollerVoltage = 0.0;

        switch (systemState) {
            case INTAKE: {
                rollerVoltage = constants.intakingVoltage;
                break;
            }
            case FEED: {
                rollerVoltage = constants.feedingVoltage;
                break;
            }
            case EJECT: {
                rollerVoltage = constants.ejectingVoltage;
                break;
            }
            case OFF:
            default:
                rollerVoltage = 0.0;
                break;
        }

        io.setTargetVoltage(rollerVoltage);

    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public record Constants(double intakingVoltage, double feedingVoltage, double ejectingVoltage) {
    }
}
