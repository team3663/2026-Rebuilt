package frc.robot.subsystems.intake;

import edu.wpi.first.units.measure.Voltage;

import edu.wpi.first.math.util.Units;

public interface IntakeIO {
    default void updateInputs(IntakeInputs inputs) {
    }

    default Intake.Constants getConstants() {
        return new Intake.Constants(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0));
    }

    default void setTargetPivotPosition(double position) {
    }

    default void setTargetPivotVoltage(double voltage) {
    }

    default void setTargetIntakeVoltage(double voltage) {
    }

    default void resetPivotPosition(double position) {
    }

    default void sysIdPivot(Voltage voltage) {
    }

    default void stopPivot() {
        setTargetPivotVoltage(0.0);
    }

    default void stopIntake() {
        setTargetIntakeVoltage(0.0);
    }
}
