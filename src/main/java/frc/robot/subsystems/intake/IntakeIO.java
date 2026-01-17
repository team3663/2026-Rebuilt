package frc.robot.subsystems.intake;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs){

    }

    default void setTargetPosition(double position) {
    }

    default void setTargetVoltagePivot(double voltage) {
    }

    default void setTargetVoltageIntake(double voltage) {
    }

    default void resetPivotPosition(double position){

    }

    default void stopPivot() {
        setTargetVoltagePivot(0.0);
    }

    default void stopIntake() {
        setTargetVoltageIntake(0.0);
    }
}
