package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakePivot;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    private final HoodIO io;
    private final Constants constants;
    private final HoodInputsAutoLogged inputs = new HoodInputsAutoLogged();

    private final double ZEROING_THRESHOLD = 0.25;

    private WantedState wantedState = WantedState.IDLE;
    private SystemState systemState = SystemState.NON_ZEROED;

    private boolean isZeroed = false;
    private double lastNonZero = Timer.getFPGATimestamp();
    private double targetShootingPosition = 0.0;

    public Hood(HoodIO io){
        this.io = io;
        this.constants = io.getConstants();
    }

    public enum WantedState {
        IDLE,
        SHOOTING,
        STOW
    }

    public enum SystemState {
        NON_ZEROED,
        IDLE,
        SHOOTING,
        STOW
    }

    public void periodic(){
        io.updateInputs(inputs);

        systemState = handleStateTransition();

        Logger.processInputs("Shooter/Hood/Inputs", inputs);
        Logger.recordOutput("Shooter/Hood/WantedState", wantedState);
        Logger.recordOutput("Shooter/Hood/SystemState", systemState);
        Logger.recordOutput("Shooter/Hood/LastNonZero", lastNonZero);
        Logger.recordOutput("Shooter/Hood/TargetShootingPosition", targetShootingPosition);

        applyState();
    }

    private SystemState handleStateTransition(){
        return switch (wantedState) {
            case IDLE:
                if (isZeroed) yield SystemState.IDLE;
                else yield SystemState.NON_ZEROED;
            case STOW:
                if (isZeroed) yield SystemState.STOW;
                else yield SystemState.NON_ZEROED;
            case SHOOTING:
                if (isZeroed) yield SystemState.SHOOTING;
                else yield SystemState.NON_ZEROED;
        };
    }

    private void applyState(){
        double hoodPosition = 0.0;

        switch (systemState) {
            case NON_ZEROED: {
                isZeroed = false;
                io.setVoltage(-1.5);
                if (lastNonZero + ZEROING_THRESHOLD <= Timer.getFPGATimestamp()) {
                    io.stop();
                    io.resetPosition(constants.minimumHoodPosition);
                    isZeroed = true;
                    // Set the intake to DEFAULT right after zeroing
                    setSystemState(systemState.IDLE);
                }
                break;
            }
            case STOW: {
                hoodPosition = constants.minimumHoodPosition;
                break;
            }
            case SHOOTING: {
                hoodPosition = Units.degreesToRadians(hoodPosition);
                break;
            }
            case IDLE: {
                hoodPosition = 0.0;
                break;
            }
        }

        io.setPosition(hoodPosition);
    }

    private void setSystemState(SystemState systemState) {
        this.systemState = systemState;
    }

    public void setWantedState(WantedState wantedState) {
        this.wantedState = wantedState;
    }

    public boolean getIsZeroed() {
        return isZeroed;
    }

    public void setTargetShootingPosition(double position) {
        this.targetShootingPosition = position;
    }

    /**
     * @param maximumHoodPosition
     * @param minimumHoodPosition
     */
    public record Constants(
            double maximumHoodPosition,
            double minimumHoodPosition) {}
}
