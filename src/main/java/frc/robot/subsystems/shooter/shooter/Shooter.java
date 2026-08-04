package frc.robot.subsystems.shooter.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.shooter.ShooterInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
    private final Constants constants;
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.IDLE;

    private double targetShootingVelocity = 0.0;

    public Shooter(ShooterIO io){
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        IDLE,
        DEFAULT,
        SHOOTING
    }

    public enum SystemState {
        IDLE,
        DEFAULT,
        SHOOTING
    }

    public void periodic(){
        io.updateInputs(inputs);

        systemState = handleStateTransition();

        Logger.processInputs("Shooter/Shooter/Inputs", inputs);
        Logger.recordOutput("Shooter/Shooter/WantedState", wantedState);
        Logger.recordOutput("Shooter/Shooter/SystemState", systemState);

        applyState();
    }

    public SystemState handleStateTransition(){
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLE;
            case DEFAULT:
                yield SystemState.DEFAULT;
            case SHOOTING:
                yield SystemState.SHOOTING;
        };
    }

    public void applyState() {
        double shooterVelociy = 0.0;

        switch (systemState) {
            case IDLE: {
                shooterVelociy = 0.0;
                break;
            }
            case SHOOTING: {
                shooterVelociy = Units.rotationsPerMinuteToRadiansPerSecond(targetShootingVelocity);
                break;
            }
            case DEFAULT: {
                shooterVelociy = constants.defaultVelocity;
                break;
            }
        }

        io.setTargetVelocity(shooterVelociy);
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public void setTargetShootingVelocity(double targetShootingVelocity) {
        this.targetShootingVelocity = targetShootingVelocity;
    }

    /**
     * @param defaultVelocity
     */
    public record Constants(double defaultVelocity){}
}
