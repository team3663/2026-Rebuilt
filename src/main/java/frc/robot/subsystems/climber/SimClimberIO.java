package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Robot;

public class SimClimberIO implements ClimberIO {
    private final double DEPLOY_GEAR__RATIO = 1.0;
    private final double CLIMB_GEAR_RATIO = 1.0;

    private final double CLIMB_SPROCKET_RADIUS = Units.inchesToMeters(2);


    private final double ROBOT_MASS = 50;

    private final DCMotor climbMotor = DCMotor.getKrakenX60(1);

    boolean stopped = false;

    private final ElevatorSim climbSim = new ElevatorSim(
            climbMotor,
            CLIMB_GEAR_RATIO,
            ROBOT_MASS,
            CLIMB_SPROCKET_RADIUS,
            0.0,
            3.0,
            true,
            0.0
            );

    private final PIDController climbController = new PIDController(0.0, 0.0, 0.0);

    @Override
    public void updateInputs(ClimberInputs inputs) {
        double voltage = climbController.calculate(climbSim.getPositionMeters());
        if (!stopped) {
            climbSim.setInputVoltage(voltage);
        }
        inputs.currentClimbMotorVelocity = climbSim.getVelocityMetersPerSecond();
        //outputs rotations instead of meters
        inputs.currentClimbMotorPosition = climbSim.getPositionMeters() / CLIMB_SPROCKET_RADIUS * 2 * Math.PI;
        inputs.currentClimbMotorAppliedCurrent = climbSim.getCurrentDrawAmps();
        inputs.currentClimbMotorAppliedVoltage = voltage;

        climbSim.update(Robot.defaultPeriodSecs);
    }

    @Override
    public void setTargetClimbPosition(double degrees) {
        climbController.setSetpoint(degrees * 2 * Math.PI * CLIMB_SPROCKET_RADIUS);
    }

    @Override
    public void setTargetClimbVoltage(double voltage) {
        climbSim.setInputVoltage(voltage);
    }

    @Override
    public void stopClimb() {
        stopped = true;
        climbSim.setInputVoltage(0.0);
    }
}
