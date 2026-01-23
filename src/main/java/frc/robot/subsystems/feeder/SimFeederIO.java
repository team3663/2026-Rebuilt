package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANrangeSimState;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Robot;

public class SimFeederIO implements FeederIO{

    private final TalonFX motor1; //14;
    private final TalonFX motor2; //15;
    private final DCMotorSim sim = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1),
                    0.001, 1.0),
            DCMotor.getKrakenX60(1).withReduction(1.0));
//    private final CANrangeSimState CANrangeSim = new CANrangeSimState();
    private double targetVelocity = Double.NaN;
    private double targetFeederVoltage = Double.NaN;
    public SimFeederIO(TalonFX motor1, TalonFX motor2) {
        this.motor1 = motor1;
        this.motor2 = motor2;

    }

    public final ProfiledPIDController controller = new ProfiledPIDController(
            1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(
            Units.rotationsPerMinuteToRadiansPerSecond(500),
            Units.rotationsPerMinuteToRadiansPerSecond(700)
    )
    );

    @Override
    public void updateInputs(FeederInputs inputs){
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

}
