package frc.robot.subsystems.climber;

public interface ClimberIO {
    default void updateInputs(ClimberInputs inputs) {}
    //deploy motor control
    default void setTargetDeployPosition(double degrees) {}

    default void setTargetDeployVoltage(double voltage) {}

    default void resetDeployPosition(double position) {}

    //climb motor control
    default void setTargetClimbPosition(double degrees) {}

    default void setTargetClimbVoltage(double voltage) {}

    default void resetClimbPosition(double position) {}

    //stoppers
    default void stopClimb() {setTargetClimbVoltage(0.0);}

    default void stopDeploy() {setTargetDeployVoltage(0.0);}
}