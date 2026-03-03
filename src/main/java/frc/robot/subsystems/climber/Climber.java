package frc.robot.subsystems.climber;


import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class Climber extends SubsystemBase {
    private final double MAX_CLIMB_HEIGHT = Units.inchesToMeters(9.0);
    private final double MIN_CLIMB_HEIGHT = 0.0;
    private final double CLIMB_HEIGHT = Units.inchesToMeters(4.0);
    private final double VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(0.1);
    private final double POSITION_THRESHOLD = Units.inchesToMeters(0.1);

    private final ClimberIO io;
    private final ClimberInputsAutoLogged inputs = new ClimberInputsAutoLogged();


    private boolean Zeroed = false;
    private double targetPosition;
    private double targetVoltage;
    private double targetVelocity;


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.recordOutput("Climber/HoodZeroed", Zeroed);
        Logger.recordOutput("Climber/TargetPosition", targetPosition);
        Logger.recordOutput("Climber/TargetVelocity", targetVelocity);
        Logger.recordOutput("Climber/TargetVoltage", targetVoltage);
        Logger.processInputs("Climber/Inputs", inputs);
    }
    public Climber(ClimberIO io){
        this.io = io;
    }
    public Command goToWithPosition(double position){
        return run(()->
                io.setTargetPosition(position)).until(() -> atTargetPosition());
    }


    public Command goToWithVoltage(double voltage){
        return runEnd(()->
                io.setTargetVoltage(voltage), ()-> io.stop());}

    public Command deploy(){
        return run(()->
                io.setTargetPosition(MAX_CLIMB_HEIGHT)).until(() -> atTargetPosition());
    }
    public Command climb(){
        return run(()->
                io.setTargetPosition(CLIMB_HEIGHT)).until(() -> atTargetPosition());
    }


    public Command zeroClimber(){
        return runEnd(()->
                        io.setTargetVoltage(-1.0), ()-> {
                    io.stop();
                    Zeroed = true;
                }
        ).until(()-> inputs.currentClimbVelocity <= VELOCITY_THRESHOLD);
    }
    public Command stop(){
        return runOnce(() -> io.stop());}
    public boolean atTargetPosition(){
        return Math.abs(targetPosition - inputs.currentClimbPosition) <= POSITION_THRESHOLD;
    }
    public boolean atTargetVoltage(){
        return targetVoltage == inputs.currentClimbVoltage;
    }


}
