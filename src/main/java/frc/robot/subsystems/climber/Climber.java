package frc.robot.subsystems.climber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Climber extends SubsystemBase {
    private static final double HOOKS_POSITION_THRESHOLD = Units.inchesToMeters(0.1);
    private static final double CLIMB_POSITION_THRESHOLD = Units.inchesToMeters(0.1);
    //TODO when its figured out add the distance the elevator has to travel after the upper hook makes contact to clear the rung
    public static final double LEVEL_1_CLIMB_HEIGHT = Units.inchesToMeters(22.0);

            ;
    public static final double LEVEL_2_CLIMB_HEIGHT = Units.inchesToMeters(19.0);
    public static final double LEVEL_3_CLIMB_HEIGHT = Units.inchesToMeters(19.1);

    //TODO when its figured out add the distance the elevator has to travel after the lower hook makes contact to clear the rung
    public static final double ELEVATOR_BASE_TO_LOWER_HOOKS_POSITION = Units.inchesToMeters(21.0);
    public static final double HALF_LIFT = Units.inchesToMeters(10);


    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private final Constants constants;

    private double targetHooksVoltage;
    private double targetClimbVoltage;

    //TODO will need to include both hooks target positions
    private double targetHooksPosition;
    private double targetClimbPosition;

    private boolean hooksZeroed = false;
    private boolean climbZeroed = false;
    public boolean hooksDeployed = false;

    public Climber(ClimberIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber/inputs", inputs);
        Logger.recordOutput("Climber/targetHooksVoltage", targetHooksVoltage);
        Logger.recordOutput("CLimber/targetHooksPosition", targetHooksPosition);
        Logger.recordOutput("Climber/hooksVelocity", hooksZeroed);

        Logger.recordOutput("Climber/targetClimbVoltage", targetClimbVoltage);
        Logger.recordOutput("CLimber/targetClimbPosition", targetClimbPosition);
        Logger.recordOutput("Climber/climbVelocity", climbZeroed);

    }

    /**DEPLOY HOOKS**/
    public Command deployHooksWithVoltage(double voltage) {
        return runEnd(() -> {
            targetHooksVoltage = voltage;
            io.setTargetUpperHooksVoltage(voltage);
            io.setTargetLowerHooksVoltage(voltage);
            },
                () -> {
            io.stopUpperHooks();
            io.stopLowerHooks();});
    }

    public Command zeroHooks(){
        return runOnce(() -> {
            io.setTargetUpperHooksVoltage(-1.0);
            io.setTargetLowerHooksVoltage(-1.0);
        }).until(() -> inputs.currentUpperHooksMotorVelocity < Units.rotationsPerMinuteToRadiansPerSecond(0.1) &&
                inputs.currentLowerHooksMotorVelocity < Units.rotationsPerMinuteToRadiansPerSecond(0.1)
        ).andThen(() -> io.resetHooksPosition(0.0, 0.0));
    }

    public Command deployHooks() {
        return runOnce(() -> {
            io.setTargetUpperHooksVoltage(1.0);
            io.setTargetLowerHooksVoltage(1.0);
        }).until(() -> inputs.currentUpperHooksMotorVelocity < Units.rotationsPerMinuteToRadiansPerSecond(0.1) &&
                inputs.currentLowerHooksMotorVelocity < Units.rotationsPerMinuteToRadiansPerSecond(0.1)
        ).andThen(() -> hooksDeployed = true);
    }

    public double getTargetHooksVoltage() {
        return targetHooksVoltage;
    }

    public double getTargetHooksPosition() {
        return targetHooksPosition;
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
        }, io::stopClimb).withDeadline(waitUntil(() -> Math.abs(inputs.currentClimbMotor1Velocity) < Units.rotationsPerMinuteToRadiansPerSecond(0.01))
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
    public boolean isDeployed(){
        return elevatorAtPosition(LEVEL_1_CLIMB_HEIGHT) && hooksAtPosition(Units.degreesToRadians(90.0), Units.degreesToRadians(90.0));
    }

    public Command fullClimb(double targetClimbHeight) {
        return runOnce(() -> {
            io.goToPosition(LEVEL_1_CLIMB_HEIGHT);
            deployHooks();
            io.goToPosition(-HALF_LIFT);
        }).andThen(() -> {
            if (targetClimbHeight != LEVEL_1_CLIMB_HEIGHT) {
                io.goToPosition(-ELEVATOR_BASE_TO_LOWER_HOOKS_POSITION + HALF_LIFT);
                io.goToPosition(LEVEL_2_CLIMB_HEIGHT);
                io.goToPosition(LEVEL_2_CLIMB_HEIGHT - HALF_LIFT);
            }
        }).andThen(() -> {
            if (targetClimbHeight == LEVEL_3_CLIMB_HEIGHT) {
                io.goToPosition(-ELEVATOR_BASE_TO_LOWER_HOOKS_POSITION + HALF_LIFT);
                io.goToPosition(LEVEL_3_CLIMB_HEIGHT);
                io.goToPosition(-ELEVATOR_BASE_TO_LOWER_HOOKS_POSITION);
            }
        });
    }

    public Command autoClimb() {
        return runOnce(() -> {
            io.goToPosition(LEVEL_1_CLIMB_HEIGHT);
            deployHooks();
            io.goToPosition(-HALF_LIFT);
                }
        );
    }

    public Command releaseClimb(){
        return runOnce(() -> {
           io.goToPosition(Units.inchesToMeters(24.0));
           io.retractHooks();
           this.zeroClimbMotors();
        });
    }

    public Command stop() {
        return runOnce(() -> {
            targetHooksVoltage = 0.0;
            targetClimbVoltage = 0.0;
            targetHooksPosition = 0.0;
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