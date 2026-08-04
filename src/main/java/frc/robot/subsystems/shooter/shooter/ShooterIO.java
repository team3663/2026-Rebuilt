package frc.robot.subsystems.shooter.shooter;

public interface ShooterIO {
    default Shooter.Constants getConstants() {
        return new Shooter.Constants(0.0);
    }

    default void updateInputs(ShooterInputs inputs) {
    }

    default void stop(){}

    default void setTargetVoltage(double voltage){}

    default void setTargetVelocity(double velocity) {}
}
