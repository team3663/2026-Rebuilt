package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import static edu.wpi.first.wpilibj.TimedRobot.kDefaultPeriod;

public class SimIntakeIO implements IntakeIO {
    /**
     * Minimum pivot angle and maximum pivot angle
     */
    private static final Intake.Constants CONSTANTS = new Intake.Constants(
            Units.degreesToRadians(0.0), Units.degreesToRadians(0.0)
    );

    // TODO Get gear ratio
    private static final double INTAKE_GEAR_RATIO = 1.0;
    private static final double INTAKE_LENGTH = 1.0;
    private static final double PIVOT_GEAR_RATIO = 1.0;
    private static final double PIVOT_MOMENT_OF_INERTIA = 0.1;
    private static final double PIVOT_STARTING_ANGLE = 0.0;

    // Get values
    private final DCMotorSim intakeSim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, INTAKE_GEAR_RATIO),
            DCMotor.getKrakenX60(1).withReduction(INTAKE_GEAR_RATIO)
    );
    private final SingleJointedArmSim pivotSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60(2), PIVOT_GEAR_RATIO, PIVOT_MOMENT_OF_INERTIA,
            INTAKE_LENGTH, CONSTANTS.minimumPivotAngle(), CONSTANTS.maximumPivotAngle(),
            true, Units.degreesToRadians(PIVOT_STARTING_ANGLE));

    // TODO, What do I set these values
    private final ProfiledPIDController intakeController = new ProfiledPIDController(
            100.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(200.0),
            Units.rotationsPerMinuteToRadiansPerSecond(400.0)
    ));

    private final ProfiledPIDController pivotController = new ProfiledPIDController(
            100.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(500.0),
            Units.rotationsPerMinuteToRadiansPerSecond(700.0)
    ));

    private double targetIntakePosition = Double.NaN;
    private double targetIntakeVoltage = Double.NaN;

    private double targetPivotPosition = Double.NaN;
    private double targetPivotVoltage = Double.NaN;

    @Override
    public Intake.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(IntakeInputs inputs) {
        double intakeVoltage = 0.0;
        if (Double.isFinite(targetIntakePosition)) {
            intakeVoltage = intakeController.calculate(intakeSim.getAngularPositionRad(), targetIntakePosition);
        } else if (Double.isFinite(targetIntakeVoltage)) {
            intakeVoltage = targetIntakeVoltage;
        }
        intakeSim.setInputVoltage(intakeVoltage);

        intakeSim.update(kDefaultPeriod);

        inputs.currentIntakeAppliedVoltage = intakeSim.getInput().get(0, 0);
        inputs.currentIntakeVelocity = intakeSim.getAngularVelocityRadPerSec();

        double pivotVoltage = 0.0;
        if (Double.isFinite(targetPivotPosition)) {
            pivotVoltage = pivotController.calculate(pivotSim.getAngleRads(), targetPivotPosition);
        } else if (Double.isFinite(targetPivotVoltage)) {
            pivotVoltage = targetPivotVoltage;
        }
        pivotSim.setInputVoltage(pivotVoltage);

        pivotSim.update(kDefaultPeriod);

        inputs.currentPivot1AppliedVoltage = pivotSim.getInput().get(0, 0);
        inputs.currentPivot1Position = pivotSim.getAngleRads();
        inputs.currentPivot1Velocity = pivotSim.getVelocityRadPerSec();


    }
}
