package frc.robot.subsystems.hopper;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.Intake;

public interface HopperIO {
    default void setTargetVoltage(double tunnelVoltage, double hopperVoltage, double rollerVoltage) {
    }

    default Hopper.Constants getConstants() {
        return new Hopper.Constants(0.0, 0.0, 0.0, 0.0);
    }

    default void stop() {
        setTargetVoltage(0.0, 0.0, 0.0);
    }

    default void updateInputs(HopperInputs inputs) {
    }
}