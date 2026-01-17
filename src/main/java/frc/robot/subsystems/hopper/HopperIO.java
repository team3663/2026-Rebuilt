package frc.robot.subsystems.hopper;

public interface HopperIO {
    default void setVoltage(double voltage) {}

    default void stop() {setVoltage(0.0);}

    default void updateInputs(HopperInputs inputs) {}
}
