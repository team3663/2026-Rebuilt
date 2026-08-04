package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
    private final Constants constants;
    private final TurretIO io;
    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

    private Turret.WantedState wantedState = Turret.WantedState.IDLE;
    private Turret.SystemState systemState = Turret.SystemState.IDLE;

    private double targetAimingPosition = 0.0;

    public Turret(TurretIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        IDLE,
        DEFAULT
    }

    public enum SystemState {
        IDLE,
        DEFAULT
    }

    public void periodic(){
        io.updateInputs(inputs);

        systemState = handleStateTransition();

        Logger.processInputs("Shooter/Turret/inputs", inputs);
        Logger.recordOutput("Shooter/Turret/WantedState", wantedState);
        Logger.recordOutput("Shooter/Turret/SystemState", systemState);

        applyState();
    }

    public SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLE;
            case DEFAULT:
                yield SystemState.DEFAULT;
        };
    }

    public void applyState() {
        double aimingPosition = 0.0;

        switch (systemState) {
            case IDLE: {
                aimingPosition = 0.0;
                break;
            }
            case DEFAULT: {
                aimingPosition = targetAimingPosition;
                break;
            }
        }

        io.setTargetPosition(Units.degreesToRadians(aimingPosition));
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setTargetAimingPosition(double position) {
        this.targetAimingPosition = position;
    }

    public record Constants(double minimumTurretPosition, double maximumTurretPosition) {}
}
