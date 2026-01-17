package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private boolean pivotZeroed = false;
    private double targetVoltage = 0.0;
    private double targetPivotPosition = 0.0;

    public Intake(IntakeIO io){
        this.io=io;
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("intake", inputs);
    }

    public Command stopPivot(){
        return runOnce(io::stopPivot);
    }

    public Command stopIntake(){
        return runOnce(io::stopIntake);
    }

    public double getPivotPosition(){
        return targetPivotPosition;
    }

    // Pivot
    public boolean isAtTargetPivotPosition(){
        return false;
    }
    //TODO add zeroing pivot
    // TODO add everything I skipped

    // Intake
    public Command intakeWithVoltage(double voltage){
        return runEnd(()-> {
                    targetVoltage = voltage;
                    io.setTargetIntakeVoltage(targetVoltage);
                }, io::stopIntake);
    }

}
