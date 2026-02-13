package frc.robot.subsystems.hopper;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class SimHopperIO implements HopperIO {

    private static final double HOPPER_GEAR_RATIO = 16.0 / 32.0;
    private static final double CORNERING_MECHANISM_GEAR_RATIO = 1.0;

    private final FlywheelSim sim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            .0001,
                            HOPPER_GEAR_RATIO), DCMotor.getKrakenX60(1));
    private final FlywheelSim corneringSim =
            new FlywheelSim(
                    LinearSystemId.createFlywheelSystem(
                            DCMotor.getKrakenX60(1),
                            .0001,
                            CORNERING_MECHANISM_GEAR_RATIO), DCMotor.getKrakenX60(1));

    @Override
    public void updateInputs(HopperInputs inputs) {
        sim.update(Robot.defaultPeriodSecs);

        inputs.curentVelocity = sim.getAngularVelocityRadPerSec();
        inputs.currentAppliedVoltage = sim.getInputVoltage();

        inputs.motorSupplyCurrent = sim.getCurrentDrawAmps();

        corneringSim.update(Robot.defaultPeriodSecs);

        inputs.corneringMotorCurentVelocity = corneringSim.getAngularVelocityRadPerSec();
        inputs.corneringMotorSupplyCurrent = corneringSim.getCurrentDrawAmps();
        inputs.corneringMotorCurrentAppliedVoltage = corneringSim.getInputVoltage();
    }

    @Override
    public void setTargetVoltage(double voltage, double corneringVoltage) {
        sim.setInputVoltage(voltage);
        corneringSim.setInputVoltage(corneringVoltage);
    }

    @Override
    public void stop() {
        sim.setInputVoltage(0.0);
        corneringSim.setInputVoltage(0.0);
    }
}