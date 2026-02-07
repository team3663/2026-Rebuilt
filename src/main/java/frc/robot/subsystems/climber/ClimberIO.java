package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;

public interface ClimberIO {
    default void updateInputs(ClimberInputs inputs) {
    }

    //deploy motor control
    default void setTargetUpperHooksPosition(double degrees) {
    }

    default void setTargetLowerHooksPosition(double degrees) {
    }

    default Climber.Constants getConstants() {
        return new Climber.Constants(Units.inchesToMeters(68.0), Units.degreesToRadians(90.0), Units.degreesToRadians(180.0));
    }

    default void setTargetUpperHooksVoltage(double voltage) {
    }

    default void setTargetLowerHooksVoltage(double voltage) {
    }

    default void resetHooksPosition(double upperHooksPosition, double lowerHooksPosition) {
    }

    default void deployHooks() {
    }

    default void retractHooks() {
    }

    //climb motor control
    default void setTargetClimbPosition(double degrees) {
    }

    default void setTargetClimbVoltage(double voltage) {
    }

    default void resetClimbPosition(double position) {
    }

    default void goToPosition(double position) {
    }

    default void zeroClimber() {
    }

    //stoppers
    default void stopClimb() {
        setTargetClimbVoltage(0.0);
    }

    default void stopUpperHooks() {
        setTargetUpperHooksVoltage(0.0);
    }

    default void stopLowerHooks() {
        setTargetLowerHooksVoltage(0.0);
    }

}