package frc.robot.subsystems.intake;

public interface IntakePivotIO {

    default void updateInputs(IntakePivotInputs pivotInputs) {
    }

    default IntakePivot.Constants getConstants() {
        return new IntakePivot.Constants(0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0);
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
