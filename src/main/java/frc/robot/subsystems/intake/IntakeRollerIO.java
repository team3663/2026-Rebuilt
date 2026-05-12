package frc.robot.subsystems.intake;

public interface IntakeRollerIO {

    default void updateInputs(IntakeInputsAutoLogged rollerInputs) {
    }

    default void stop(){}

    default void setTargetVoltage(double angle){}

}
