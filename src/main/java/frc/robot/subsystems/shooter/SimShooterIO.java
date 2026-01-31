package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class SimShooterIO implements ShooterIO {
    private static final Shooter.Constants CONSTANTS = new Shooter.Constants(0, Units.degreesToRadians(90), Units.degreesToRadians(-180), Units.degreesToRadians(180));

    private static final double HOOD_GEAR_RATIO = 1.0;
    private static final double TURRET_GEAR_RATIO = 50.0;
    private static final double SHOOTER_GEAR_RATIO = 1.0;
    private static final double STDEV = 0.0;

    private final DCMotor hoodMotor = DCMotor.getFalcon500Foc(1);
    private final DCMotor turretMotor = DCMotor.getFalcon500Foc(1);
    private final DCMotor shooterMotors = DCMotor.getFalcon500Foc(2);

    private final DCMotorSim hoodSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(hoodMotor, 0.01, HOOD_GEAR_RATIO), hoodMotor, STDEV, STDEV);

    private final ProfiledPIDController hoodController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.rotationsPerMinuteToRadiansPerSecond(500.0), Units.rotationsPerMinuteToRadiansPerSecond(700.0)));

    private final DCMotorSim turretSim = new DCMotorSim(LinearSystemId.createDCMotorSystem(turretMotor, 1.0, TURRET_GEAR_RATIO), turretMotor, STDEV, STDEV);

    private final ProfiledPIDController turretController = new ProfiledPIDController(10.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.rotationsPerMinuteToRadiansPerSecond(500.0), Units.rotationsPerMinuteToRadiansPerSecond(700.0)));

    private final FlywheelSim shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(shooterMotors, 0.1, SHOOTER_GEAR_RATIO), shooterMotors, STDEV);

    private final ProfiledPIDController shooterController = new ProfiledPIDController(1.0, 0.0, 0.0, new TrapezoidProfile.Constraints(Units.rotationsPerMinuteToRadiansPerSecond(500.0), Units.rotationsPerMinuteToRadiansPerSecond(700.0)));


    private double targetHoodPosition = Double.NaN;
    private double targetHoodVoltage = Double.NaN;

    private double targetTurretPosition = Double.NaN;
    private double targetTurretVoltage = Double.NaN;

    private double targetShooterVelocity = Double.NaN;
    private double targetShooterVoltage = Double.NaN;

    @Override
    public Shooter.Constants getConstants() {
        return CONSTANTS;
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        double hoodVoltage = 0.0;
        if (Double.isFinite(targetHoodPosition)) {
            hoodVoltage = hoodController.calculate(hoodSim.getAngularPositionRad(), targetHoodPosition);
        } else if (Double.isFinite(targetHoodVoltage)) {
            hoodVoltage = targetHoodVoltage;
        }
        hoodSim.setInputVoltage(hoodVoltage);

        hoodSim.update(Robot.defaultPeriodSecs);

        inputs.currentHoodAppliedVoltage = hoodSim.getInput().get(0, 0);
        inputs.currentHoodPosition = hoodSim.getAngularPositionRad();
        inputs.currentHoodVelocity = hoodSim.getAngularVelocityRadPerSec();

        double turretVoltage = 0.0;
        if (Double.isFinite(targetTurretPosition)) {
            turretVoltage = turretController.calculate(turretSim.getAngularPositionRad(), targetTurretPosition);
        } else if (Double.isFinite(targetTurretVoltage)) {
            turretVoltage = targetTurretVoltage;
        }
        turretSim.setInputVoltage(turretVoltage);

        turretSim.update(Robot.defaultPeriodSecs);

        inputs.currentTurretAppliedVoltage = turretSim.getInput().get(0, 0);
        inputs.currentTurretPosition = turretSim.getAngularPositionRad();
        inputs.currentTurretVelocity = turretSim.getAngularVelocityRadPerSec();

        double shooterVoltage = 0.0;
        if (Double.isFinite(targetShooterVelocity)) {
            shooterVoltage = shooterController.calculate(shooterSim.getAngularVelocityRadPerSec(), targetShooterVelocity);
        } else if (Double.isFinite(targetShooterVoltage)) {
            shooterVoltage = targetShooterVoltage;
        }
        shooterSim.setInputVoltage(shooterVoltage);

        shooterSim.update(Robot.defaultPeriodSecs);

        inputs.currentShooterAppliedVoltage1 = shooterSim.getInput().get(0, 0);
        inputs.currentShooterVelocity1 = shooterSim.getAngularVelocityRadPerSec();
    }

    @Override
    public void setHoodTargetPosition(double position) {
        targetHoodPosition = position;
        targetHoodVoltage = Double.NaN;
    }

    @Override
    public void setHoodTargetVoltage(double voltage) {
        targetHoodPosition = Double.NaN;
        targetHoodVoltage = voltage;
    }

    @Override
    public void setTurretTargetPosition(double position) {
        targetTurretPosition = position;
        targetTurretVoltage = Double.NaN;
    }

    @Override
    public void setTurretTargetVoltage(double voltage) {
        targetTurretPosition = Double.NaN;
        targetTurretVoltage = voltage;
    }

    @Override
    public void setShooterTargetVelocity(double velocity) {
        targetShooterVelocity = velocity;
        targetShooterVoltage = Double.NaN;
    }

    @Override
    public void setShooterTargetVoltage(double voltage) {
        targetShooterVelocity = Double.NaN;
        targetShooterVoltage = voltage;
    }
}
