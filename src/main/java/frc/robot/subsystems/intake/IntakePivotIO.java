package frc.robot.subsystems.intake;

public interface IntakePivotIO {

    default void updateInputs(IntakePivotInputs pivotInputs) {
    }

    default void stop() {
    }

    default void resetPosition(double angle) {
    }

    default void setTargetPosition(double angle) {
    }

    default void setTargetVoltage(double voltage) {
    }
}
