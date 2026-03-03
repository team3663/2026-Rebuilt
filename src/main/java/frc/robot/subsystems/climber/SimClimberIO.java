package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

public class SimClimberIO implements ClimberIO {

    private static final ElevatorSim sim = new ElevatorSim(LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0.001, 1.0), DCMotor.getKrakenX60(1).withReduction(1.0),  0.0, Units.inchesToMeters(9.0), true, 0.0, 0.0, 0.0);
    public final ProfiledPIDController controller = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.rotationsPerMinuteToRadiansPerSecond(500), Units.rotationsPerMinuteToRadiansPerSecond(700)));
    private double targetVoltage = Double.NaN;
    private double targetPosition = Double.NaN;

    @Override
    public void updateInputs(ClimberInputs inputs) {
        double climberVoltage = 0.0;
        if (Double.isFinite(targetPosition)) {
            climberVoltage = controller.calculate(sim.getPositionMeters(), targetPosition);
        } else if (Double.isFinite(targetVoltage)) {
            climberVoltage = targetVoltage;
        }
        sim.setInputVoltage(climberVoltage);

        sim.update(Robot.defaultPeriodSecs);

        inputs.currentClimbVoltage = sim.getInput().get(0, 0);
        inputs.currentClimbPosition = sim.getPositionMeters();
    }

    @Override
    public void setTargetVoltage(double voltage) {
        targetVoltage = voltage;
        targetPosition = Double.NaN;
    }

    @Override
    public void setTargetPosition(double position) {
        targetPosition = position;
        targetVoltage = Double.NaN;
    }
}
