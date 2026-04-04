package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class SimHopperIO implements HopperIO {

    private static final double HOPPER_GEAR_RATIO = 16.0 / 32.0;

    private final FlywheelSim sim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            .0001,
                            HOPPER_GEAR_RATIO), DCMotor.getKrakenX60(1));

    @Override
    public void updateInputs(HopperInputs inputs) {
        sim.update(Robot.defaultPeriodSecs);

        inputs.currentHopperVelocity = sim.getAngularVelocityRadPerSec();
        inputs.currentHopperAppliedVoltage = sim.getInputVoltage();

        inputs.hopperSupplyCurrent = sim.getCurrentDrawAmps();
    }

    @Override
    public void setTargetVoltage(double corneringTargetVoltage, double hopperTargetVoltage) {
        sim.setInputVoltage(corneringTargetVoltage);
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0.0);
    }
}