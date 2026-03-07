package frc.robot.subsystems.climber;


import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.climber.ClimberInputs;
import frc.robot.subsystems.shooter.Shooter;


public interface ClimberIO {
    default Climber.Constants getConstants() {
        return new Climber.Constants( Units.inchesToMeters(9.0), 0.0, Units.inchesToMeters(5.0));
    }

    default void updateInputs(ClimberInputs inputs) {
    }


    default void stop() {
        setTargetVoltage(0.0);
    }


    default void setTargetVoltage(double voltage) {
    }


    default void setTargetPosition(double position) {
    }
}