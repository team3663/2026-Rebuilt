// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.ControllerHelper;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Drive extends SubsystemBase {
    private static final double DISTANCE_THRESHOLD = Units.inchesToMeters(1.5);

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final double driveBaseRadius;
    private final LinearVelocity maxLinearVelocity;

    private final SwerveDriveKinematics kinematics;
    private final SwerveDrivePoseEstimator poseEstimator;

    private Rotation2d rawGyroRotation = Rotation2d.kZero;
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[]{
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition(),
                    new SwerveModulePosition()
            };

    @AutoLogOutput(key = "Drive/TargetPose")
    private Pose2d targetPose = null;

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontLeftModuleConstants,
            ModuleIO frModuleIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> frontRightModuleConstants,
            ModuleIO blModuleIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backLeftModuleConstants,
            ModuleIO brModuleIO,
            SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> backRightModuleConstants) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, frontLeftModuleConstants);
        modules[1] = new Module(frModuleIO, 1, frontRightModuleConstants);
        modules[2] = new Module(blModuleIO, 2, backLeftModuleConstants);
        modules[3] = new Module(brModuleIO, 3, backRightModuleConstants);

        kinematics = new SwerveDriveKinematics(
                new Translation2d(frontLeftModuleConstants.LocationX, frontLeftModuleConstants.LocationY),
                new Translation2d(frontRightModuleConstants.LocationX, frontRightModuleConstants.LocationY),
                new Translation2d(backLeftModuleConstants.LocationX, backLeftModuleConstants.LocationY),
                new Translation2d(backRightModuleConstants.LocationX, backRightModuleConstants.LocationY)
        );
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.kZero);

        driveBaseRadius =
                Math.max(
                        Math.max(
                                kinematics.getModules()[0].getNorm(),
                                kinematics.getModules()[1].getNorm()),
                        Math.max(
                                kinematics.getModules()[2].getNorm(),
                                kinematics.getModules()[3].getNorm()));
        maxLinearVelocity =
                MetersPerSecond.of(
                        Math.min(
                                Math.min(
                                        modules[0].getConstants().SpeedAt12Volts,
                                        modules[1].getConstants().SpeedAt12Volts),
                                Math.min(
                                        modules[2].getConstants().SpeedAt12Volts,
                                        modules[3].getConstants().SpeedAt12Volts)));

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[]{});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[]{});
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxLinearVelocity);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /**
     * Runs the drive in a straight line with the specified drive output.
     */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /**
     * Stops the drive.
     */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
     * return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            var constants = modules[i].getConstants();
            headings[i] = new Translation2d(constants.LocationX, constants.LocationY).getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /**
     * Returns a command to run a quasistatic test in the specified direction.
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /**
     * Returns a command to run a dynamic test in the specified direction.
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the modules.
     */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /**
     * Returns the measured chassis speeds of the robot.
     */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Returns the position of each module in radians.
     */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /**
     * Returns the average velocity of the modules in rotations/sec (Phoenix native units).
     */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /**
     * Returns the current odometry pose.
     */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    /**
     * Returns the current odometry rotation.
     */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    public boolean atPosition(Translation2d target, double threshold) {
        return target.minus(getPose().getTranslation()).getNorm() < threshold;
    }

    public boolean atPosition(Translation2d target) {
        return atPosition(target, DISTANCE_THRESHOLD);
    }

    @AutoLogOutput(key = "Drive/AtTargetPosition")
    public boolean atTargetPosition() {
        return targetPose != null && atPosition(targetPose.getTranslation());
    }

    /**
     * Adds a new timestamped vision measurement.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        poseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    /**
     * Returns the maximum linear speed in meters per sec.
     */
    public double getMaxLinearSpeedMetersPerSec() {
        return maxLinearVelocity.in(MetersPerSecond);
    }

    /**
     * Returns the maximum angular speed in radians per sec.
     */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / driveBaseRadius;
    }

    public double getDriveBaseRadius() {
        return driveBaseRadius;
    }

    public Command drive(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        return run(() -> runVelocity(chassisSpeedsSupplier.get()));
    }

    /**
     * Drives the robot to a given pose fromm the robot's current position using a pathplanner path
     *
     * @param targetPose of where you want the robot to go
     * @return follows a pathplanner path command
     */
    public Command goToPosition(Supplier<Pose2d> targetPose, BooleanSupplier slowAccel, Supplier<Double> maxVelocity) {
        PIDController controller = new PIDController(7.0, 0.0, 1.0);
        PIDController rotationController = new PIDController(10.0, 0.0, 0.0);
        rotationController.enableContinuousInput(-Math.PI, Math.PI);

        return runOnce(
                () -> {
                    controller.reset();
                    controller.setP(slowAccel.getAsBoolean() ? 5.0 : 6.0);
                    rotationController.reset();
                    controller.setSetpoint(0.0);
                    rotationController.setSetpoint(targetPose.get().getRotation().getRadians());
                })
                .andThen(drive(() ->
                        {
                            var target = targetPose.get();
                            this.targetPose = target;

                            var current = getPose();
                            var error = target.getTranslation().minus(current.getTranslation());
                            var linearVelocity = MathUtil.clamp(
                                    controller.calculate(error.getNorm()),
                                    -maxVelocity.get(),
                                    maxVelocity.get());
                            var velocity = new Translation2d(-linearVelocity, error.getAngle());
                            var angularVel = rotationController.calculate(
                                    current.getRotation().getRadians(), target.getRotation().getRadians());

                            return ChassisSpeeds.fromFieldRelativeSpeeds(
                                    velocity.getX(),
                                    velocity.getY(),
                                    angularVel,
                                    current.getRotation()
                            );
                        }
                ));
    }

    public Command goToPosition(Supplier<Pose2d> targetPose, BooleanSupplier slowAccel) {
        return goToPosition(targetPose, slowAccel, this::getMaxLinearSpeedMetersPerSec);
    }

    public Command resetOdometry(Supplier<Pose2d> poseSupplier) {
        return Commands.runOnce(() -> poseEstimator.resetPose(poseSupplier.get()));
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public Command joystickDrive(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return drive(
                () -> {// Apply deadband
                    double x = xSupplier.getAsDouble();
                    double y = ySupplier.getAsDouble();

                    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), ControllerHelper.DEADBAND);
                    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

                    // Square magnitude for more precise control
                    linearMagnitude = linearMagnitude * linearMagnitude;

                    // Return new linear velocity
                    Translation2d linearVelocity = new Pose2d(Translation2d.kZero, linearDirection)
                            .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                            .getTranslation();

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), ControllerHelper.DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * getMaxLinearSpeedMetersPerSec(),
                                    omega * getMaxAngularSpeedRadPerSec());
                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
                    return
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? getRotation().plus(new Rotation2d(Math.PI))
                                            : getRotation());
                });
    }
}
