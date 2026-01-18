package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Intake extends SubsystemBase {
    public static final double POSITION_THRESHOLD = Units.degreesToRadians(2.0);

    private final IntakeIO io;
    private final Constants constants;
    private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

    private boolean pivotZeroed = false;
    private double targetVoltage = 0.0;
    private double targetPivotPosition = 0.0;

    public Intake(IntakeIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);

        //Logging
        Logger.recordOutput("Intake/PivotZeroed", pivotZeroed);
        Logger.recordOutput("Intake/TargetVoltage", targetVoltage);
        Logger.recordOutput("Intake/TargetPivotPosition", targetPivotPosition);
        Logger.processInputs("Intake/Inputs", inputs);
    }

    // Pivot
    public Command stopPivot() {
        return runOnce(io::stopPivot);
    }

    public double getPivotPosition() {
        return targetPivotPosition;
    }

    public boolean isAtPosition(double position, double threshold) {
        return Math.abs(inputs.currentPivot1Position - position) < threshold;
    }

    public double getValidPivotPosition(double position) {
        return Math.max(constants.minimumPivotAngle, Math.min(constants.maximumPivotAngle, position));
    }

    /**
     * @param position DoubleSuppplier
     * @return a run command to move the pivot to a target position
     */
    public Command followPivotPositions(DoubleSupplier position) {
        return run(() -> {
            if (pivotZeroed) {
                targetPivotPosition = getValidPivotPosition(getPivotPosition());
                io.setTargetPivotPosition(targetPivotPosition);
            }
        });
    }

    /**
     * @return run end command to zero the pivot on a hard stop
     */
    public Command zeroPivot() {
        return runEnd(() -> {
            io.setTargetPivotVoltage(-1.5);
            io.setTargetPivotPosition(constants.minimumPivotAngle);
        }, io::stopPivot)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentPivot1Velocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(() -> {
                            io.resetPivotPosition(constants.minimumPivotAngle);
                            pivotZeroed = true;
                        }));
    }


    // Intake
    public Command stopIntake() {
        return runOnce(io::stopIntake);
    }

    public Command eject() {
        return runEnd(() -> intakeWithVoltage(-2.0), this::stopIntake);
    }

    /**
     * @param voltage
     * @return run end command that runs the intake motor
     */
    public Command intakeWithVoltage(double voltage) {
        return runEnd(() -> {
            targetVoltage = voltage;
            io.setTargetIntakeVoltage(targetVoltage);
        }, io::stopIntake);
    }

    public record Constants(double minimumPivotAngle, double maximumPivotAngle) {

    }

}
