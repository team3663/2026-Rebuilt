package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class IntakePivot extends SubsystemBase {
    private final C2026IntakePivotIO io;
    private Constants constants;
    private final IntakePivotInputsAutoLogged inputs = new IntakePivotInputsAutoLogged();

    private WantedState wantedState = WantedState.OFF;
    private SystemState systemState = SystemState.OFF;

    private boolean isZeroed = false;
    private

    public IntakePivot(C2026IntakePivotIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        STOW,
        DEPLOY,
        FEED,
        ZEROING,
        DEFAULT,
        OFF
    }

    public enum SystemState {
        STOW,
        DEPLOY,
        FEED,
        ZEROING,
        DEFAULT,
        OFF
    }

    public void periodic() {
        io.updateInputs(inputs);
        if (inputs.currentPivotVelocity != 0.0) {
            var currentTime = System.currentTimeMillis();
        }

        //Logging
        Logger.recordOutput("Intake/Pivot/WantedState", wantedState);
        Logger.recordOutput("Intake/Pivot/SystemState", systemState);
        Logger.processInputs("Intake/Pivot/Inputs", inputs);
    }

    private SystemState handleStateTransition() {
        return switch (wantedState) {
            case FEED: {
                if (isZeroed) yield SystemState.FEED;
                else yield SystemState.OFF;
            }
            case DEPLOY: {
                if (isZeroed) yield SystemState.DEPLOY;
                else yield SystemState.OFF;
            }
            case STOW: {
                if (isZeroed) yield SystemState.STOW;
                else yield SystemState.OFF;
            }
            case ZEROING: {
                yield SystemState.ZEROING;
            }
            default:
                yield SystemState.OFF;
        };
    }

    private void applyState() {
        double pivotTargetPosition = 0.0;

        switch (systemState) {
            case DEPLOY: {
                pivotTargetPosition = constants.deployAngle;
            }
            case FEED: {
                pivotTargetPosition = constants.feedingAngle;
            }
            case STOW: {
                pivotTargetPosition = constants.stowAngle;
            }








            case ZEROING: {
                // TODO - DON'T USE COMMANDS - Track time when it was last non-zero; looking at if that variable plus a
                    // TODO wait is less than a certain amount
                isZeroed = false;
                io.setTargetVoltage(-1.5);
                if (Math.abs(inputs.currentPivotVelocity) < 0.01) {
                    io.stop();
                    io.resetPosition(constants.minimumPivotAngle);
                    isZeroed = true;
                }











                // Deploy the intake right after zeroing
                setWantedState(WantedState.DEFAULT);
            }






            default:
                pivotTargetPosition = inputs.currentPivotPosition;
        }

        io.setTargetPosition(pivotTargetPosition);
    }


    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
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
