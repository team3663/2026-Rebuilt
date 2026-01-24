package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase {
    private final IntakeIO io;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private final boolean pivotZeroed = false;
    private double targetVoltage = 0.0;
    private final double targetPivotPosition = 0.0;

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Intake/PivotZeroed", pivotZeroed);
        Logger.recordOutput("Intake/TargetVoltage", targetVoltage);
        Logger.recordOutput("Intake/TargetPivotPosition", targetPivotPosition);
        Logger.processInputs("Intake/Inputs", inputs);
    }

    public Command stopPivot() {
        return runOnce(io::stopPivot);
    }

    public Command stopIntake() {
        return runOnce(io::stopIntake);
    }

    public double getPivotPosition() {
        return targetPivotPosition;
    }

    // Pivot
    public boolean isAtTargetPivotPosition() {
        return false;
    }
    //TODO add zeroing pivot
    // TODO add everything I skipped

    // Intake
    public Command intakeWithVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetIntakeVoltage(targetVoltage);
        }, io::stopIntake);
    }
}
