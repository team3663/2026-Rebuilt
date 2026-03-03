package frc.robot.subsystems.climber;


import frc.robot.subsystems.climber.ClimberInputs;


public interface ClimberIO {
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