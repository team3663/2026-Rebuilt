package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;

public interface ShooterIO {
    default Shooter.Constants getConstants() {
        return new Shooter.Constants(0, Units.degreesToRadians(90), Units.degreesToRadians(-180), Units.degreesToRadians(180));
    }

    default void updateInputs(ShooterInputs inputs) {
    }

    // Hood
    default void stopHood() {
        setHoodTargetVoltage(0.0);
    }

    default void resetHoodPosition(double position) {
    }

    default void setHoodTargetPosition(double position) {
    }

    default void setHoodTargetVoltage(double voltage) {
    }

    // Turret
    default void stopTurret() {
        setTurretTargetVoltage(0.0);
    }

    default void setTurretTargetPosition(double position) {
    }

    default void setTurretTargetVoltage(double voltage) {
    }

    // Shooter Motor
    default void stopShooter() {
        setShooterTargetVoltage(0.0);
    }

    default void setShooterTargetVelocity(double velocity) {
    }

    default void setShooterTargetVoltage(double voltage) {
    }
}