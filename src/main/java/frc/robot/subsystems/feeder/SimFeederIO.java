package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class SimFeederIO implements FeederIO {


    private static final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));
//    private final CANrangeSimState CANrangeSim = new CANrangeSimState();
    public final ProfiledPIDController controller = new ProfiledPIDController(
            1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(500),
            Units.rotationsPerMinuteToRadiansPerSecond(700)
    )
    );
    private double targetVelocity = Double.NaN;
    private double targetFeederVoltage = Double.NaN;

    @Override
    public void updateInputs(FeederInputs inputs) {
        double feederVoltage = 0.0;
        if (Double.isFinite(targetVelocity)) {
            feederVoltage = controller.calculate(sim.getAngularVelocityRadPerSec(), targetVelocity);
        } else if (Double.isFinite(targetFeederVoltage)) {
            feederVoltage = targetFeederVoltage;
        }
        sim.setInputVoltage(feederVoltage);

        sim.update(Robot.defaultPeriodSecs);

        inputs.currentAppliedVoltage = sim.getInput().get(0, 0);
        inputs.currentVelocity = sim.getAngularVelocityRadPerSec();
    }

    @Override
    public void setTargetVoltage(double voltage) {
        targetFeederVoltage = voltage;
    }

    @Override
    public void setTargetVelocity(double velocity) {
        targetVelocity = velocity;
    }
}
