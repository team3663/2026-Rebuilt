package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeInputs;
import frc.robot.subsystems.intake.IntakePivot;

public interface HoodIO {
    default void updateInputs(HoodInputs inputs) {
    }

    default Hood.Constants getConstants() {
        return new Hood.Constants(0.0, 0.0);
    }

    default void stop(){}

    default void resetPosition(double position) {}

    default void setVoltage(double volts){}

    default void setPosition(double angle) {}

}
