package frc.robot.subsystems.shooter;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FiringSolution;
import org.littletonrobotics.junction.Logger;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class Shooter extends SubsystemBase {
    private final static double HOOD_POSITION_THRESHOLD = Units.degreesToRadians(1);
    private final static double TURRET_POSITION_THRESHOLD = Units.degreesToRadians(1);
    private final static double SHOOTER_VELOCITY_THRESHOLD = Units.rotationsPerMinuteToRadiansPerSecond(1);

    private final static double DEFAULT_SHOOTER_VELOCITY = Units.rotationsPerMinuteToRadiansPerSecond(3500.0);

    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private final Constants constants;

    private boolean hoodZeroed = false;
    private double targetHoodPosition;
    private double targetTurretPosition;
    private double targetShooterVelocity;

    public Shooter(ShooterIO io) {
        this.io = io;
        this.constants = io.getConstants();
    }

    public Constants getConstants() {
        return constants;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.recordOutput("Shooter/HoodZeroed", hoodZeroed);
        Logger.recordOutput("Shooter/TargetHoodPosition", targetHoodPosition);
        Logger.recordOutput("Shooter/TargetTurretPosition", targetTurretPosition);
        Logger.recordOutput("Shooter/TargetShooterVelocity", targetShooterVelocity);
        Logger.processInputs("Shooter/Inputs", inputs);
    }

    public Command stop() {
        return runOnce(() -> {
            targetHoodPosition = 0.0;
            targetTurretPosition = 0.0;
            targetShooterVelocity = 0.0;
            io.stopHood();
            io.stopTurret();
            io.stopShooter();
        });
    }

    public boolean atTargetPositions() {
        return this.atHoodTargetPosition() && this.atTurretTargetPosition();
    }

    public boolean atTargets() {
        return atTargetPositions() && atShooterTargetVelocity();
    }

    public boolean atPositions(double hoodPosition, double turretPosition) {
        return this.hoodAtPosition(hoodPosition) && this.turretAtPosition(turretPosition);
    }

    public boolean isAt(double hoodPosition, double turretPosition, double shooterVelocity) {
        return this.hoodAtPosition(hoodPosition) && this.turretAtPosition(turretPosition) && this.shooterAtVelocity(shooterVelocity);
    }

    public Command goToDefaultState() {
        return follow(() -> this.getConstants().minimumHoodPosition(), () -> Units.degreesToRadians(-180.0), () -> DEFAULT_SHOOTER_VELOCITY);
    }

    public Command goTo(double hoodPosition, double turretPosition, double shooterVelocity) {
        return runEnd(() -> {
            // Hood
            if (hoodZeroed) {
                targetHoodPosition = getValidHoodPosition(hoodPosition);
                io.setHoodTargetPosition(targetHoodPosition);
            }

            // Turret
            targetTurretPosition = getNearestTargetTurretAngle(turretPosition);
            io.setTurretTargetPosition(targetTurretPosition);

            // Shooter
            targetShooterVelocity = shooterVelocity;
            io.setShooterTargetVelocity(targetShooterVelocity);
        }, this::stop).until(this::atTargetPositions);
    }

    public Command follow(DoubleSupplier hoodPosition, DoubleSupplier turretPosition, DoubleSupplier shooterVelocity) {
        return run(() -> {
            // Hood
            if (hoodZeroed) {
                targetHoodPosition = getValidHoodPosition(hoodPosition.getAsDouble());
                io.setHoodTargetPosition(targetHoodPosition);
            }

            // Turret
            targetTurretPosition = getNearestTargetTurretAngle(turretPosition.getAsDouble());
            io.setTurretTargetPosition(targetTurretPosition);

            // Shooter
            targetShooterVelocity = shooterVelocity.getAsDouble();
            io.setShooterTargetVelocity(targetShooterVelocity);
        });
    }

    public Command follow(Supplier<FiringSolution> firingSolution) {
        return follow(() -> firingSolution.get().hoodAngle(), () -> firingSolution.get().turretAngle(), () -> firingSolution.get().shooterVelocity());
    }

    // Hood
    public double getHoodVelocity() {
        return inputs.currentHoodVelocity;
    }

    public double getHoodPosition() {
        return inputs.currentHoodPosition;
    }

    public boolean atHoodTargetPosition() {
        return hoodAtPosition(targetHoodPosition, HOOD_POSITION_THRESHOLD);
    }

    public boolean hoodAtPosition(double position) {
        return hoodAtPosition(position, HOOD_POSITION_THRESHOLD);
    }

    public boolean hoodAtPosition(double position, double threshold) {
        return Math.abs(inputs.currentHoodPosition - position) < threshold;
    }

    public double getTargetHoodPosition() {
        return targetHoodPosition;
    }

    private double getValidHoodPosition(double position) {
        return Math.max(constants.minimumHoodPosition, Math.min(constants.maximumHoodPosition, position));
    }

    public Command zeroHood() {
        return runEnd(() -> {
            io.setHoodTargetVoltage(-1.5);
            targetHoodPosition = constants.minimumHoodPosition;
        }, io::stopHood)
                .withDeadline(waitUntil(() -> Math.abs(inputs.currentHoodVelocity) < 0.01)
                        .beforeStarting(waitSeconds(0.25))
                        .andThen(() -> {
                            io.resetHoodPosition(constants.minimumHoodPosition);
                            hoodZeroed = true;
                        }));
    }

    // Turret
    public double getTurretVelocity() {
        return inputs.currentTurretVelocity;
    }

    public double getTurretPosition() {
        return inputs.currentTurretPosition;
    }

    public boolean atTurretTargetPosition() {
        return turretAtPosition(targetTurretPosition, TURRET_POSITION_THRESHOLD);
    }

    public boolean turretAtPosition(double position) {
        return turretAtPosition(position, TURRET_POSITION_THRESHOLD);
    }

    public boolean turretAtPosition(double position, double threshold) {
        return Math.abs(inputs.currentTurretPosition - position) < threshold;
    }

    public double getTargetTurretPosition() {
        return targetTurretPosition;
    }

    private double getNearestTargetTurretAngle(double target) {
        double current = getValidTurretPosition(inputs.currentTurretPosition);

        target = getSmallestEquivalentAngle(target);
        double reducedCurrent = getSmallestEquivalentAngle(current);
        double fullTarget = target + (current - reducedCurrent);

        if (fullTarget > constants.maximumTurretPosition)
            return fullTarget - 2 * Math.PI;
        if (fullTarget < constants.minimumTurretPosition)
            return fullTarget + 2 * Math.PI;
        return fullTarget;
    }

    // Returns the angle reduced to be equivalent and <= Math.PI and > -Math.PI
    private double getSmallestEquivalentAngle(double angle) {
        double modded = angle % (2 * Math.PI);
        if (modded > Math.PI) return modded - 2 * Math.PI;
        if (modded <= -Math.PI) return  modded + 2 * Math.PI;
        return modded;
    }

    private double getValidTurretPosition(double position) {
        return Math.max(constants.minimumTurretPosition, Math.min(constants.maximumTurretPosition, position));
    }

    // Shooter
    public double getShooterVelocity() {
        return inputs.currentShooterVelocity1;
    }

    public boolean atShooterTargetVelocity() {
        return shooterAtVelocity(targetShooterVelocity, SHOOTER_VELOCITY_THRESHOLD);
    }

    public boolean shooterAtVelocity(double position) {
        return shooterAtVelocity(position, SHOOTER_VELOCITY_THRESHOLD);
    }

    public boolean shooterAtVelocity(double position, double threshold) {
        return Math.abs(inputs.currentShooterVelocity1 - position) < threshold;
    }

    public double getTargetShooterVelocity() {
        return targetShooterVelocity;
    }

    public record Constants(
            double minimumHoodPosition,
            double maximumHoodPosition,
            double minimumTurretPosition,
            double maximumTurretPosition
    ) {}
}