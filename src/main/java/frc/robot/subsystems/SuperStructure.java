package frc.robot.subsystems;

import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.IntakeSuperStructure;
import frc.robot.subsystems.shooter.ShooterSuperStructure;
import org.littletonrobotics.junction.Logger;


public class SuperStructure {
    private final ShooterSuperStructure shooter;
    private final IntakeSuperStructure intake;
    private final Feeder feeder;
    private final Hopper hopper;

    private SuperStructure.WantedState wantedState = SuperStructure.WantedState.IDLE;
    private SuperStructure.SystemState systemState = SuperStructure.SystemState.IDLE;

    public SuperStructure(ShooterSuperStructure shooter, IntakeSuperStructure intake, Feeder feeder, Hopper hopper) {
        this.shooter = shooter;
        this.intake = intake;
        this.feeder = feeder;
        this.hopper = hopper;
    }

    public enum WantedState {
        IDLE,
        DEFAULT,
        STOW,
        INTAKING,
        EJECTING,
        SHOOTING_INTAKE_RAISED,
        SHOOTING_INTAKE_DOWN,
        GOING_OVER_BUMP,
        SHOOTING_WHILE_INTAKING
    }

    public enum SystemState {
        IDLE,
        DEFAULT,
        STOW,
        INTAKING,
        EJECTING,
        SHOOTING_INTAKE_RAISED,
        SHOOTING_INTAKE_DOWN,
        GOING_OVER_BUMP,
        SHOOTING_WHILE_INTAKING
    }

    public void periodic() {
        systemState = handleStateTransition();

        Logger.recordOutput("SuperStructure/WantedState", wantedState);
        Logger.recordOutput("SuperStructure/SystemState", systemState);

        applyState();
    }

    public SystemState handleStateTransition() {
        return switch (wantedState) {
            case IDLE:
                yield SystemState.IDLE;
            case DEFAULT:
                yield SystemState.DEFAULT;
            case STOW:
                yield SystemState.STOW;
            case EJECTING:
                yield SystemState.EJECTING;
            case SHOOTING_INTAKE_RAISED:
                yield SystemState.SHOOTING_INTAKE_RAISED;
            case SHOOTING_INTAKE_DOWN:
                yield SystemState.SHOOTING_INTAKE_DOWN;
            case INTAKING:
                yield SystemState.INTAKING;
            case GOING_OVER_BUMP:
                yield SystemState.GOING_OVER_BUMP;
            case SHOOTING_WHILE_INTAKING:
                yield SystemState.SHOOTING_WHILE_INTAKING;
        };
    }

    public void applyState() {
        switch (systemState) {
            case IDLE: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.IDLE);
                intake.setWantedState(IntakeSuperStructure.WantedState.IDLE);
                feeder.setWantedState(Feeder.WantedState.OFF);
                hopper.setWantedState(Hopper.WantedState.IDLE);
            }
            case DEFAULT: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.DEFAULT);
                intake.setWantedState(IntakeSuperStructure.WantedState.DEFAULT);
                feeder.setWantedState(Feeder.WantedState.DEFAULT);
                hopper.setWantedState(Hopper.WantedState.IDLE);
            }
            case STOW: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.DEFAULT);
                intake.setWantedState(IntakeSuperStructure.WantedState.STOW);
                feeder.setWantedState(Feeder.WantedState.DEFAULT);
                hopper.setWantedState(Hopper.WantedState.IDLE);
            }
            case EJECTING: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.DEFAULT);
                intake.setWantedState(IntakeSuperStructure.WantedState.EJECT);
                feeder.setWantedState(Feeder.WantedState.DEFAULT);
                hopper.setWantedState(Hopper.WantedState.EJECTING);
            }
            case INTAKING: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.DEFAULT);
                intake.setWantedState(IntakeSuperStructure.WantedState.INTAKE);
                feeder.setWantedState(Feeder.WantedState.DEFAULT);
                hopper.setWantedState(Hopper.WantedState.IDLE);
            }
            case SHOOTING_INTAKE_RAISED: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.SHOOTING);
                intake.setWantedState(IntakeSuperStructure.WantedState.DEFAULT);
                feeder.setWantedState(Feeder.WantedState.FEED);
                hopper.setWantedState(Hopper.WantedState.FEEDING);
            }
            case SHOOTING_INTAKE_DOWN: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.SHOOTING);
                intake.setWantedState(IntakeSuperStructure.WantedState.DEPLOY);
                feeder.setWantedState(Feeder.WantedState.FEED);
                hopper.setWantedState(Hopper.WantedState.FEEDING);
            }
            case SHOOTING_WHILE_INTAKING: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.SHOOTING);
                intake.setWantedState(IntakeSuperStructure.WantedState.INTAKE);
                feeder.setWantedState(Feeder.WantedState.FEED);
                hopper.setWantedState(Hopper.WantedState.FEEDING);
            }
            case GOING_OVER_BUMP: {
                shooter.setWantedState(ShooterSuperStructure.WantedState.STOWED_HOOD);
                intake.setWantedState(IntakeSuperStructure.WantedState.DEFAULT);
                feeder.setWantedState(Feeder.WantedState.DEFAULT);
                hopper.setWantedState(Hopper.WantedState.IDLE);
            }
        }
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean isZeroed() {
        return (shooter.getIsZeroed()) && (intake.getIsZeroed());
    }



}
