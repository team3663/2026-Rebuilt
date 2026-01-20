package frc.robot.subsystems.feeder;

public interface FeederIO {
    default void updateInputs(FeederInputs inputs) {
    }

    default void stop() {
        setTargetVoltage(0.0);
    }

    default void setTargetVoltage(double voltage) {
    }

    default void setTargetVelocity(double velocity) {
    }
}
