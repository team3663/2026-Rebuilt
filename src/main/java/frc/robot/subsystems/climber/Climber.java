package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Climber extends SubsystemBase {
    private static final double DEPLOY_POSITION_THRESHOLD = 2;
    private static final double CLIMB_POSITION_THRESHOLD = 2;

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();

    private final double MIN_DEPLOY_POSITION = 0.0;
    private final double MAX_DEPLOY_POSITION = 360.0;

    private final double MIN_CLIMB_POSITION = 0.0;
    private final double MAX_CLIMB_POSITION = 360.0;

    private final double EXTENDED_DEPLOY_POSITION = 360.0;
    private final double EXTENDED_CLIMB_POSITION = 360.0;

    private double targetDeployVoltage;
    private double targetClimbVoltage;

    private double targetDeployPosition;
    private double targetClimbPosition;

    private boolean deployZeroed = false;
    private boolean climbZeroed = false;

    public Climber(ClimberIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/inputs", inputs);
        Logger.recordOutput("Climber/deployVoltage", targetDeployVoltage);
        Logger.recordOutput("CLimber/deployPosition", targetDeployPosition);
        Logger.recordOutput("Climber/deployVelocity", deployZeroed);

        Logger.recordOutput("Climber/climbVoltage", targetClimbVoltage);
        Logger.recordOutput("CLimber/climbPosition", targetClimbPosition);
        Logger.recordOutput("Climber/climbVelocity", climbZeroed);
    }

    //Deploy
    public Command deployWithVoltage(double voltage) {
        return runEnd(() -> {
            targetDeployVoltage = voltage;
            io.setTargetDeployVoltage(voltage);
            }, io::stopDeploy);
    }

    public boolean atDeployPosition() {
        return Math.abs(targetDeployPosition - inputs.currentDeployMotorPosition) < DEPLOY_POSITION_THRESHOLD;
    }

    public Command Deploy() {
        return runEnd(() ->{
            if (deployZeroed) {
                targetDeployPosition = EXTENDED_DEPLOY_POSITION;
                io.setTargetDeployPosition(getValidDeployPosition(EXTENDED_DEPLOY_POSITION));
            }
        },() -> {stop();}).until(this::atDeployPosition);
    }

    public Command zeroDeploy() {
        return runEnd(() -> {
            io.setTargetDeployVoltage(-1.5);
            targetDeployPosition = MIN_DEPLOY_POSITION;
        }, io::stopDeploy)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentDeployMotorVelocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(() -> {
                            io.resetDeployPosition(MIN_DEPLOY_POSITION);
                            deployZeroed = true;
                        }));
    }

    public double getValidDeployPosition(double position) {
        if (position > MIN_DEPLOY_POSITION && position < MAX_DEPLOY_POSITION) {
            return position;
        } else {
            return MIN_DEPLOY_POSITION;
        }
    }

    public double getTargetDeployVoltage() {
        return targetDeployVoltage;
    }

    public double getTargetDeployPosition() {
        return targetDeployPosition;
    }

    //Climb
    public Command climbWithVoltage(double voltage) {
        return runEnd(() -> {
            targetClimbVoltage = voltage;
            io.setTargetClimbVoltage(voltage);
        }, io::stopClimb);
    }

    public boolean atClimbPosition() {
        return Math.abs(targetClimbPosition - inputs.currentClimbMotorPosition) < CLIMB_POSITION_THRESHOLD;
    }

    public Command Climb() {
        return runEnd(() ->{
            if (climbZeroed) {
                targetClimbPosition = getValidClimbPosition(EXTENDED_CLIMB_POSITION);
                io.setTargetClimbPosition(getValidClimbPosition(EXTENDED_CLIMB_POSITION));
            }
        },() -> {stop();}).until(this::atClimbPosition);
    }

    public Command zeroClimb() {
        return runEnd(() -> {
            io.setTargetClimbVoltage(-1.5);
            targetClimbPosition = MIN_CLIMB_POSITION;
        }, io::stopClimb).withDeadline(waitUntil(() -> Math.abs(inputs.currentClimbMotorVelocity) < 0.01)
                .beforeStarting(waitSeconds(.25))
                .andThen(() -> {
                    io.resetClimbPosition(MIN_CLIMB_POSITION);
                    climbZeroed = true;
                }));
    }

    public double getValidClimbPosition(double position) {
        if (position > MIN_CLIMB_POSITION && position < MAX_CLIMB_POSITION) {
            return position;
        } else {
            return MIN_CLIMB_POSITION;
        }
    }

    public double getTargetClimbVoltage() {
        return targetClimbVoltage;
    }

    public double getTargetClimbPosition() {
        return targetClimbPosition;
    }

    public Command stop() {
        return runOnce(() -> {
            targetDeployVoltage = 0.0;
            targetClimbVoltage = 0.0;
            targetDeployPosition = 0.0;
            targetClimbPosition = 0.0;
            io.stopDeploy();
            io.stopClimb();
        });
    }
}