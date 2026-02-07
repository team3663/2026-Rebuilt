package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class SimClimberIO implements ClimberIO {
    private static final Climber.Constants CONSTANTS = new Climber.Constants(1.0, Math.PI/2, Math.PI);

    private final double DEPLOY_GEAR__RATIO = 1.0;
    private final double CLIMB_GEAR_RATIO = 1.0;

    //deploy constants
    private final double DEPLOY_ARM_LENGTH = Units.inchesToMeters(18);
    private final double MIN_DEPLOY_ANGLE = Units.degreesToRadians(0.0);
    private final double MAX_DEPLOY_ANGLE = Units.degreesToRadians(90.0);
    private final DCMotor deployMotor = DCMotor.getKrakenX60(1);

    //climb constants
    private final double CLIMB_SPROCKET_RADIUS = Units.inchesToMeters(2);
    private final double ROBOT_MASS = 50;
    private final DCMotor climbMotor = DCMotor.getKrakenX60(1);

    boolean deployStopped = false;
    boolean climbStopped = false;

    boolean deployUsingVoltage;
    boolean climbUsingVoltage;

    private double  targetDeployVoltage;
    private double targetClimbVoltage;

    //deploy simulation
    private final SingleJointedArmSim deploySim = new SingleJointedArmSim(
            climbMotor,
            CLIMB_GEAR_RATIO,
            0.1,
            DEPLOY_ARM_LENGTH,
            MIN_DEPLOY_ANGLE,
            MAX_DEPLOY_ANGLE,
            true,
            MIN_DEPLOY_ANGLE
            );

    //climbing simulation
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

    private final PIDController deployController = new PIDController(1.0, 0.0, 0.0);
    private final PIDController climbController = new PIDController(1.0, 0.0, 0.0);

    @Override
    public void updateInputs(ClimberInputs inputs) {
        //deploy inputs
        if (!deployUsingVoltage) {
            targetDeployVoltage = deployController.calculate(deploySim.getAngleRads());
        }

        if (!deployStopped) {
            deploySim.setInputVoltage(targetDeployVoltage);
        }

        inputs.currentUpperHooksMotorVelocity = deploySim.getVelocityRadPerSec();
        inputs.currentUpperHooksMotorPosition = deploySim.getAngleRads();
        inputs.currentUpperHooksMotorAppliedCurrent = deploySim.getCurrentDrawAmps();
        inputs.currentClimbMotor1AppliedVoltage = targetDeployVoltage;

        //climb inputs
        if (!climbUsingVoltage) {
            targetClimbVoltage = climbController.calculate(climbSim.getPositionMeters());
        }

        if (!climbStopped) {
            climbSim.setInputVoltage(targetClimbVoltage);
        }

        inputs.currentClimbMotor1Velocity = climbSim.getVelocityMetersPerSecond();
        inputs.currentClimbMotor1Position = climbSim.getPositionMeters();
        inputs.currentClimbMotor1AppliedCurrent = climbSim.getCurrentDrawAmps();
        inputs.currentClimbMotor1AppliedVoltage = targetClimbVoltage;

        climbSim.update(Robot.defaultPeriodSecs);
    }

    @Override
    public Climber.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void setTargetLowerHooksPosition(double degrees) {
        deployStopped = false;
        deployUsingVoltage = false;
        deployController.setSetpoint(Units.degreesToRadians(degrees));
    }

    @Override
    public void setTargetUpperHooksPosition(double degrees) {
        deployStopped = false;
        deployUsingVoltage = false;
        deployController.setSetpoint(Units.degreesToRadians(degrees));
    }

    @Override
    public void setTargetUpperHooksVoltage(double voltage) {
        deployStopped = false;
        deployUsingVoltage = true;
        deploySim.setInputVoltage(voltage);
    }
    @Override
    public void setTargetLowerHooksVoltage(double voltage) {
        deployStopped = false;
        deployUsingVoltage = true;
        deploySim.setInputVoltage(voltage);
    }
    @Override
    public void stopUpperHooks() {
        deployStopped = true;
        deploySim.setInputVoltage(0.0);
    }
    @Override
    public void stopLowerHooks() {
        deployStopped = true;
        deploySim.setInputVoltage(0.0);
    }

    @Override
    public void setTargetClimbPosition(double degrees)  {
        climbStopped = false;
        climbUsingVoltage = false;
        climbController.setSetpoint(degrees * 2 * Math.PI * CLIMB_SPROCKET_RADIUS);
    }

    @Override
    public void setTargetClimbVoltage(double voltage) {
        climbStopped = false;
        climbUsingVoltage = true;
        climbSim.setInputVoltage(voltage);
    }

    @Override
    public void stopClimb() {
        climbStopped = true;
        climbSim.setInputVoltage(0.0);
    }
}
