package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

public class IntakeSuperStructure {
    private final IntakePivot intakePivot;
    private final IntakeRoller intakeRoller;

    private IntakeSuperStructure.WantedState wantedState = IntakeSuperStructure.WantedState.IDLE;
    private IntakeSuperStructure.SystemState systemState = IntakeSuperStructure.SystemState.IDLE;

    public IntakeSuperStructure(IntakeRoller intakeRoller, IntakePivot intakePivot) {
        this.intakePivot = intakePivot;
        this.intakeRoller = intakeRoller;
    }

    public enum WantedState {
        IDLE,
        DEFAULT,
        STOW,
        DEPLOY,
        INTAKE,
        EJECT
    }

    public enum SystemState {
        IDLE,
        DEFAULT,
        STOW,
        DEPLOY,
        INTAKE,
        EJECT
    }

    public void periodic() {
        systemState = handleStateTransition();

        Logger.recordOutput("Intake/SuperStructure/WantedState", wantedState);
        Logger.recordOutput("Intake/SuperStructure/SystemState", systemState);

        applyState();
    }

    public SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLE;
            case INTAKE:
                yield SystemState.INTAKE;
            case STOW:
                yield SystemState.STOW;
            case DEPLOY:
                yield SystemState.DEPLOY;
            case DEFAULT:
                yield SystemState.DEFAULT;
            case EJECT:
                yield SystemState.EJECT;
        };
    }

    public void applyState() {
        switch(systemState) {
            case IDLE: {
                intakePivot.setWantedState(IntakePivot.WantedState.IDLE);
                intakeRoller.setWantedState(IntakeRoller.WantedState.IDLE);
                break;
            }
            case DEFAULT: {
                intakePivot.setWantedState(IntakePivot.WantedState.DEFAULT);
                intakeRoller.setWantedState(IntakeRoller.WantedState.IDLE);
                break;
            }
            case STOW: {
                intakePivot.setWantedState(IntakePivot.WantedState.STOW);
                intakeRoller.setWantedState(IntakeRoller.WantedState.IDLE);
                break;
            }
            case DEPLOY: {
                intakePivot.setWantedState(IntakePivot.WantedState.DEPLOY);
                intakeRoller.setWantedState(IntakeRoller.WantedState.IDLE);
                break;
            }
            case INTAKE: {
                intakePivot.setWantedState(IntakePivot.WantedState.FEED);
                intakeRoller.setWantedState(IntakeRoller.WantedState.INTAKE);
                break;
            }
            case EJECT: {
                intakePivot.setWantedState(IntakePivot.WantedState.FEED);
                intakeRoller.setWantedState(IntakeRoller.WantedState.EJECT);
                break;
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean getIsZeroed() {
        return intakePivot.getIsZeroed();
    }
}
