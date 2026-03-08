package frc.robot.subsystems.hopper;

public interface HopperIO {
    default void setTargetVoltage(double corneringVoltage, double hopperVoltage) {
    }

    default void stop() {
        setTargetVoltage(0.0, 0.0);
    }

    default void updateInputs(HopperInputs inputs) {
    }
}