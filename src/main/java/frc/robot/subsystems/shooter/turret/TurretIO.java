package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterInputs;

public interface TurretIO {
    default Turret.Constants getConstants() {
        return new Turret.Constants();
    }

    default void updateInputs(TurretInputs inputs) {
    }

    default void stop() {}

    default void setTargetPosition(double position) {}

    default void setTargetVoltage(double voltage) {}

}
