package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Climber extends SubsystemBase {
    private static final double HOOKS_POSITION_THRESHOLD = 0.2;
    private static final double CLIMB_POSITION_THRESHOLD = 0.2;
    //TODO double check the values for how much robot has to pull itself up by
    public static final double LEVEL_1_CLIMB_HEIGHT = 20.5;
    public static final double LEVEL_2_CLIMB_HEIGHT = 19.0;
    public static final double LEVEL_3_CLIMB_HEIGHT = 19.1;

    //this is the amount that the elevator will pull itself up in order for the lower hooks to latch so it is subject to change
    public static final double DISTANCE_FROM_ELEVATOR_BASE_TO_LOWER_HOOKS_LATCH_POSITION = 21.0;


    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final Constants constants;

    private double targetDeployVoltage;
    private double targetClimbVoltage;

    //TODO will need to include both hooks target positions
    private double targetDeployPosition;
    private double targetClimbPosition;

    private boolean deployZeroed = false;
    private boolean climbZeroed = false;
    public boolean deployed = false;

    public Climber(ClimberIO io) {
        this.io = io;
        this.constants = io.getConstants();
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

    /**DEPLOY HOOKS**/
    public Command deployHooksWithVoltage(double voltage) {
        return runEnd(() -> {
            targetDeployVoltage = voltage;
            io.setTargetUpperHooksVoltage(voltage);
            io.setTargetLowerHooksVoltage(voltage);
            },
                () -> {
            io.stopUpperHooks();
            io.stopLowerHooks();});
    }


    //TODO if using position establish this method
    public Command deployHooks() {
        return null;
    }
    //TODO figure out if lower/upper hooks have a hard stop on either end
    public Command zeroDeploy() {
        return runEnd(() -> {
            io.setTargetUpperHooksVoltage(-1.5);
            io.setTargetLowerHooksVoltage(-1.5);
            targetDeployPosition = 0.0;
        }, () -> {
                io.stopUpperHooks();
                io.stopLowerHooks();})
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentUpperHooksMotorVelocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(() -> {
                            io.resetHooksPosition(0.0, 0.0);
                            deployZeroed = true;
                        }));
    }

    public double getTargetDeployVoltage() {
        return targetDeployVoltage;
    }

    public double getTargetDeployPosition() {
        return targetDeployPosition;
    }

    /**CLIMB**/
    public Command climbWithVoltage(double voltage) {
        return runEnd(() -> {
            targetClimbVoltage = voltage;
            io.setTargetClimbVoltage(voltage);
        }, io::stopClimb);
    }
    public boolean elevatorAtPosition(double position){
        return Math.abs(position - inputs.currentClimbMotor1Position) < CLIMB_POSITION_THRESHOLD;
    }

    public boolean hooksAtPosition(double lowerPosition, double upperPosition) {
        return Math.abs(lowerPosition - inputs.currentLowerHooksMotorPosition) < HOOKS_POSITION_THRESHOLD &&
                Math.abs(upperPosition - inputs.currentUpperHooksMotorPosition) < HOOKS_POSITION_THRESHOLD;
    }

    public Command zeroClimbMotors() {
        return runEnd(() -> {
            io.setTargetClimbVoltage(-1.5);
            targetClimbPosition = 0.0;
        }, io::stopClimb).withDeadline(waitUntil(() -> Math.abs(inputs.currentClimbMotor1Velocity) < 0.01)
                .beforeStarting(waitSeconds(.25))
                .andThen(() -> {
                    io.resetClimbPosition(0.0);
                    climbZeroed = true;
                }));
    }

    public double getTargetClimbPosition() {
        return targetClimbPosition;
    }
    public Command deploy(){
        return runOnce(()->{
                    io.goToPosition(LEVEL_1_CLIMB_HEIGHT);
                    io.deployHooks();
                });
    }
    public boolean getDeployed(){
        return elevatorAtPosition(LEVEL_1_CLIMB_HEIGHT) && hooksAtPosition(180.0, 90.0);
    }

    public Command fullClimb(double targetClimbHeight) {
        return runOnce(() -> {
            io.goToPosition(-10.0);
        }).andThen(() -> {
            if (targetClimbHeight != LEVEL_1_CLIMB_HEIGHT) {
                io.goToPosition(-DISTANCE_FROM_ELEVATOR_BASE_TO_LOWER_HOOKS_LATCH_POSITION + 10.0);
                io.goToPosition(LEVEL_2_CLIMB_HEIGHT);
                io.goToPosition(LEVEL_2_CLIMB_HEIGHT - 10.0);
            }
        }).andThen(() -> {
            if (targetClimbHeight == LEVEL_3_CLIMB_HEIGHT) {
                io.goToPosition(-DISTANCE_FROM_ELEVATOR_BASE_TO_LOWER_HOOKS_LATCH_POSITION + 10.0);
                io.goToPosition(LEVEL_3_CLIMB_HEIGHT);
                io.goToPosition(-DISTANCE_FROM_ELEVATOR_BASE_TO_LOWER_HOOKS_LATCH_POSITION);
            }
        });
    }

    public Command autoClimb() {
        return runOnce(() -> {
            io.goToPosition(LEVEL_1_CLIMB_HEIGHT);
            deployHooks();
            io.goToPosition(-10.0);
                }
        );
    }

    public Command releaseClimb(){
        return runOnce(() -> {
           io.goToPosition(12.0);
           io.retractHooks();
           this.zeroClimbMotors();
        });
    }

    public Command stop() {
        return runOnce(() -> {
            targetDeployVoltage = 0.0;
            targetClimbVoltage = 0.0;
            targetDeployPosition = 0.0;
            targetClimbPosition = 0.0;
            io.stopUpperHooks();
            io.stopLowerHooks();
            io.stopClimb();
        });
    }

    public record Constants(
            double climberMaxHeight,
            double upperHooksMaxAngle,
            double lowerHooksMaxAngle
    ) {}
}