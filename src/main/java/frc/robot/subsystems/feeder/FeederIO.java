package frc.robot.subsystems.feeder;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.Intake;

public interface FeederIO {
    default void updateInputs(FeederInputs inputs) {}

    default Feeder.Constants getConstants() {
        return new Feeder.Constants(0.0, 0.0);
    }

    default void stop() {}

    default void setTargetVoltage(double volts) {}


}
