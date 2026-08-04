package frc.robot.subsystems.shooter;

import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.shooter.Shooter;
import org.littletonrobotics.junction.Logger;

public class ShooterSuperStructure {
    private final Shooter shooter;
    private final Turret turret;
    private final Hood hood;

    private ShooterSuperStructure.WantedState wantedState = ShooterSuperStructure.WantedState.IDLE;
    private ShooterSuperStructure.SystemState systemState = ShooterSuperStructure.SystemState.IDLE;

    public ShooterSuperStructure(Shooter shooter, Hood hood, Turret turret) {
        this.shooter = shooter;
        this.hood = hood;
        this.turret = turret;
    }

    public enum WantedState {
        IDLE,
        DEFAULT,
        SHOOTING,
        STOWED_HOOD
    }

    public enum SystemState {
        IDLE,
        DEFAULT,
        SHOOTING,
        STOWED_HOOD
    }

    public void periodic(){
        systemState = handleStateTransition();

        Logger.recordOutput("Shooter/SuperStructure/WantedState", wantedState);
        Logger.recordOutput("Shooter/SuperStructure/SystemState", systemState);

        applyState();
    }

    public SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLE;
            case DEFAULT:
                yield SystemState.DEFAULT;
            case SHOOTING:
                yield SystemState.SHOOTING;
            case STOWED_HOOD:
                yield SystemState.STOWED_HOOD;
        };
    }

    public void applyState() {
        switch (systemState) {
            case IDLE: {
                shooter.setWantedState(Shooter.WantedState.IDLE);
                turret.setWantedState(Turret.WantedState.IDLE);
                hood.setWantedState(Hood.WantedState.IDLE);
            }
            case DEFAULT: {
                shooter.setWantedState(Shooter.WantedState.DEFAULT);
                turret.setWantedState(Turret.WantedState.DEFAULT);
                hood.setWantedState(Hood.WantedState.SHOOTING);
            }
            case SHOOTING: {
                shooter.setWantedState(Shooter.WantedState.SHOOTING);
                turret.setWantedState(Turret.WantedState.DEFAULT);
                hood.setWantedState(Hood.WantedState.SHOOTING);
            }
            case STOWED_HOOD: {
                shooter.setWantedState(Shooter.WantedState.IDLE);
                turret.setWantedState(Turret.WantedState.DEFAULT);
                hood.setWantedState(Hood.WantedState.IDLE);
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean getIsZeroed() {
        return hood.getIsZeroed();
    }

}
