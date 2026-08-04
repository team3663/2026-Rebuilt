package frc.robot.subsystems.shooter.turret;

public interface TurretIO {
    default Turret.Constants getConstants() {
        return new Turret.Constants(0.0,0.0);
    }

    default void updateInputs(TurretInputs inputs) {
    }

    default void stop() {}

    default void setTargetPosition(double position) {}

    default void setTargetVoltage(double voltage) {}

}
