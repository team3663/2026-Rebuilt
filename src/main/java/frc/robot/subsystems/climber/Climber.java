package frc.robot.subsystems.climber;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {
    private final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(0.1);
    private final double POSITION_THRESHOLD = Units.inchesToMeters(0.1);

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();
    private Constants constants;


    private boolean Zeroed = false;
    private double targetPosition;
    private double targetVoltage;
    private double targetVelocity;


    public Climber(ClimberIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Climber/HoodZeroed", Zeroed);
        Logger.recordOutput("Climber/TargetPosition", targetPosition);
        Logger.recordOutput("Climber/TargetVelocity", targetVelocity);
        Logger.recordOutput("Climber/TargetVoltage", targetVoltage);
        Logger.processInputs("Climber/Inputs", inputs);
    }

    public Command goTo(double position) {
        return runEnd(() -> {
            if (Zeroed) {
                io.setTargetPosition(position);
            }
        }, io::stop).until(() -> atTargetPosition());
    }


    public Command withVoltage(double voltage) {
        return runEnd(() ->
                io.setTargetVoltage(voltage), () -> io.stop());
    }

    public Command deploy() {
        return run(() ->
                goTo(constants.MAX_CLIMB_HEIGHT));
    }

    public Command climb() {
        return run(() ->
                goTo(constants.CLIMB_HEIGHT));
    }
    public Command retract(){
        return run(()-> {
            goTo(constants.MIN_CLIMB_HEIGHT);
        });
    }


    public Command zeroClimber() {
        return runEnd(() ->
                        io.setTargetVoltage(-1.0), () -> {
                    io.stop();
                    Zeroed = true;
                }
        ).until(() -> inputs.currentClimbVelocity <= VELOCITY_THRESHOLD);
    }

    public Command stop() {
        return runOnce(() -> io.stop());
    }

    public boolean atTargetPosition() {
        return Math.abs(targetPosition - inputs.currentClimbPosition) <= POSITION_THRESHOLD;
    }

    public record Constants(
            double MAX_CLIMB_HEIGHT,
            double MIN_CLIMB_HEIGHT,
            double CLIMB_HEIGHT
    ) {
    }


}
