package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.wpilibj2.command.Commands.runEnd;

public class Feeder extends SubsystemBase {


    private final FeederIO io;
    private final FeederInputs inputs = new FeederInputs();

    public Feeder(FeederIO io) {this.io = io;}

    public double targetVoltage = 0.0;
    public double targetVelocity = 0.0;




    public double getCurrentVelocity(){return inputs.currentVelocity;}

    public double getCurrentVoltage(){return inputs.currentAppliedVoltage;}

    public Command withVoltage(double voltage){
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetVoltage(voltage);},
                io::stop
        );
    }
    public Command withVelocity(double velocity){
        return runEnd(() -> {
            targetVelocity = velocity;
            io.setTargetVelocity(velocity);},
                io::stop
        );
    }
    public Command stop(){
        return runOnce(io::stop);
    }
}
