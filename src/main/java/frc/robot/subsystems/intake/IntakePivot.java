package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class IntakePivot extends SubsystemBase {
    private final IntakePivotIO io;
    private Constants constants;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    private final double ZEROING_THRESHOLD = 0.25;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.NON_ZEROED;

    private boolean isZeroed = false;
    private double lastNonZero = Timer.getFPGATimestamp();

    public IntakePivot(IntakePivotIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        STOW,
        DEPLOY,
        FEED,
        IDLE,
        DEFAULT
    }

    public enum SystemState {
        STOW,
        DEPLOY,
        FEED,
        NON_ZEROED,
        IDLE,
        DEFAULT
    }

    public void periodic() {
        io.updateInputs(inputs);

        // Checking for the last time the pivot's velocity was not zero
        if (inputs.currentPivotVelocity != 0.0) {
            lastNonZero = Timer.getFPGATimestamp();
        }

        systemState = handleStateTransition();

        //Logging
        Logger.recordOutput("Intake/Pivot/WantedState", wantedState);
        Logger.recordOutput("Intake/Pivot/SystemState", systemState);
        Logger.recordOutput("Intake/Pivot/LastNonZero", lastNonZero);
        Logger.processInputs("Intake/Pivot/Inputs", inputs);

        applyState();
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case FEED: {
                if (isZeroed) yield SystemState.FEED;
                else yield SystemState.NON_ZEROED;
            }
            case DEPLOY: {
                if (isZeroed) yield SystemState.DEPLOY;
                else yield SystemState.NON_ZEROED;
            }
            case STOW: {
                if (isZeroed) yield SystemState.STOW;
                else yield SystemState.NON_ZEROED;
            }
            case IDLE: {
                if (isZeroed) yield SystemState.IDLE;
                else yield SystemState.NON_ZEROED;
            }
            case DEFAULT: {
                yield SystemState.DEFAULT;
            }
        };
    }

    private void applyState() {
        double pivotTargetPosition = 0.0;

        switch (systemState) {
            case DEPLOY: {
                pivotTargetPosition = constants.deployAngle;
                break;
            }
            case FEED: {
                pivotTargetPosition = constants.feedingAngle;
                break;
            }
            case STOW: {
                pivotTargetPosition = constants.stowAngle;
                break;
            }
            case DEFAULT: {
                pivotTargetPosition = constants.defaultAngle;
                break;
            }
            case IDLE: {
                break;
            }
            case NON_ZEROED: {
                isZeroed = false;
                io.setTargetVoltage(-1.5);
                if (lastNonZero + ZEROING_THRESHOLD <= Timer.getFPGATimestamp()) {
                    io.stop();
                    io.resetPosition(constants.minimumPivotAngle);
                    isZeroed = true;
                    // Set the intake to DEFAULT right after zeroing
                    setSystemState(systemState.IDLE);
                }
                break;
            }
            default:
                pivotTargetPosition = inputs.currentPivotPosition;
        }

        io.setTargetPosition(pivotTargetPosition);
    }


    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    private void setSystemState(SystemState systemState) {
        this.systemState = systemState;
    }

    public boolean getIsZeroed() {
        return isZeroed;
    }

    /**
     * @param minimumPivotAngle - maximum angle the pivot can rotate to
     * @param maximumPivotAngle - minimum angle the pivot can rotate to
     * @param positionThreshold - the number of degrees the pivot can be off from the desired amount
     * @param deployAngle       - the angle of the pivot while intaking
     * @param feedingAngle      - the angle of the pivot while feeding
     * @param stowAngle         - the angle of the pivot when it's all the way in the robot
     * @param defaultAngle      - the angle the pivot defaults to
     */
    public record Constants(double minimumPivotAngle, double maximumPivotAngle, double positionThreshold,
                            double deployAngle, double feedingAngle, double defaultAngle, double stowAngle) {
    }
}
