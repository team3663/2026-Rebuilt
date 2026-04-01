// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.FireControlSystem;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import static edu.wpi.first.wpilibj.DriverStation.Alliance;
import static edu.wpi.first.wpilibj.DriverStation.getAlliance;
import static edu.wpi.first.wpilibj2.command.Commands.*;
import static frc.robot.Constants.ENABLE_TEST_FEATURES;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // Subsystems
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;
    private final AutoPaths autoPaths;
    private final Vision vision;
    private final Led led;

    private final CommandFactory commandFactory;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    private final CommandXboxController testController;

    // Dashboard inputs
    private final LoggedDashboardChooser<AutoPaths.AutonomousMode> autoChooser;
    private final LoggedNetworkBoolean automaticPassingOverride = new LoggedNetworkBoolean("overrideSwitichingToPassing", false);

    private boolean shootingIntoHub = true;

    private boolean ranAuto = false;

    LoggedNetworkBoolean aimingAtHub = new LoggedNetworkBoolean("aimingAtHub", true);
    LoggedNetworkBoolean passing = new LoggedNetworkBoolean("passing", false);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotFactory robotFactory) {
        this.drive = robotFactory.createDrive();
        this.feeder = robotFactory.createFeeder();
        this.hopper = robotFactory.createHopper();
        this.intake = robotFactory.createIntake();
        this.shooter = robotFactory.createShooter();
        this.vision = robotFactory.createVision();
        this.led = robotFactory.createLed();

        commandFactory = new CommandFactory(drive, feeder, hopper, intake, shooter
//        , climber
        );
        this.autoPaths = new AutoPaths(drive, intake, shooter, commandFactory);


        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());

        // Set up SysId routines
//        autoChooser.addOption(
//                "Drive SysId (Quasistatic Forward)",
//                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
//        autoChooser.addOption(
//                "Drive SysId (Quasistatic Reverse)",
//                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
//        autoChooser.addOption(
//                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
//        autoChooser.addOption(
//                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        // Auto Routines
        autoChooser.addOption("CH-ShootInHub", autoPaths.middleStarting_shootIntoHub());
        autoChooser.addOption("LT-ShootInHub", autoPaths.leftStarting_shootIntoHub());
        autoChooser.addOption("RT-ShootIntoHub", autoPaths.rightStarting_shootIntoHub());
        autoChooser.addOption("RT-NZ-NZ", autoPaths.rightStarting_neutralZone_shoot_neutralZone());
        autoChooser.addOption("LT-NZ-NZ", autoPaths.leftStarting_neutralZone_shoot_neutralZone());
        autoChooser.addOption("LT-NZ-NZ-RB", autoPaths.leftStarting_neutralZone_shoot_neutralZoneToRightSide());
        autoChooser.addOption("RT-NZ-NZ-LB", autoPaths.rightStarting_neutralZone_shoot_neutralZoneToLeftSide());
//        autoChooser.addOption("RightUnderTrench2ft-NZ-Shoot-NeutralZoneLoop-Shoot", autoPaths.rightStarting_neutralZone_shoot_neutralZoneLoop_shoot());
        autoChooser.addOption("CH-ShootInHub-Outpost", autoPaths.middleStarting_shootIntoHub_outpost());
        autoChooser.addOption("RT-Outpost", autoPaths.rightStarting_outpost());
//        autoChooser.addOption("MiddleStarting-Depot", autoPaths.middleStarting_depot());

        // Configure the button bindings
        configureButtonBindings();

        shooter.setDefaultCommand(commandFactory.shooterDefault(() -> shootingIntoHub));

        vision.setDefaultCommand(
                vision.consumeVisionMeasurements(drive::addVisionMeasurements, () -> {
                            if (DriverStation.isAutonomous() && DriverStation.isDisabled() && !ranAuto) {
                                AutoPaths.AutonomousMode autonomousMode = autoChooser.get();

                                if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) ==
                                        DriverStation.Alliance.Blue) {
                                    return autonomousMode.blueStartingPosition().getRotation();
                                } else {
                                    return autonomousMode.redStartingPosition().getRotation();
                                }
                            } else {
                                return drive.getRotation();
                            }
                        })
                        .ignoringDisable(true));

        intake.setDefaultCommand(intake.pivotDefault());

        // Configure the button bindings
        configureButtonBindings();

        if (ENABLE_TEST_FEATURES) {
            testController = new CommandXboxController(2);
            configureTestBindings();
        } else testController = null;
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(drive.joystickDrive(
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX()
        ));

        RobotModeTriggers.disabled()
                .onTrue(Commands.either(vision.recordAuto(), vision.recordTeleop(), DriverStation::isAutonomous)
                        .ignoringDisable(true));

        Trigger resetFieldOrientedTrigger = controller.back();
        Trigger zeroTrigger = controller.start();

        Trigger intakeTrigger = controller.leftTrigger();
        Trigger stowIntakeTrigger = controller.leftBumper();

        Trigger shootTrigger = controller.rightTrigger();

        Trigger setPassingMode = controller.a();
        Trigger setShootingMode = controller.x();

        Trigger reverseIntakeTrigger = controller.y();

        // Reset gyro to 0° when B button is pressed
        resetFieldOrientedTrigger.onTrue(drive.resetOdometry(() ->
                new Pose2d(
                        drive.getPose().getTranslation(),
                        getAlliance().isPresent() && getAlliance().get() == Alliance.Red ?
                                Rotation2d.k180deg :
                                Rotation2d.kZero)));

        zeroTrigger.onTrue(
                Commands.parallel(
                        intake.zeroPivot(),
                        shooter.zeroHood()
                )
        );

        // general bindings for the intake
        intakeTrigger.whileTrue(intake.deployAndIntake());
        stowIntakeTrigger.whileTrue(intake.stow());
        reverseIntakeTrigger.whileTrue(intake.intakeAndPivot(-4.0, Intake.DEPLOY_ANGLE));

        // general bindings for the shooter
        shootTrigger.whileTrue(run(() -> {
            var shouldOverride = automaticPassingOverride.getAsBoolean();
            if (!shouldOverride) {
                shootingIntoHub = commandFactory.isHubShootingMode();
                aimingAtHub.set(commandFactory.isHubShootingMode());
                passing.set(!commandFactory.isHubShootingMode());
            }
        })).and(() -> shootingIntoHub).whileTrue(
                commandFactory.aim(shootingIntoHub)
        );

        shootTrigger.whileTrue(
                sequence(
                        waitSeconds(0.1),
                        waitUntil(commandFactory::shouldShoot),
                        repeatingSequence(
                                commandFactory.feedIntoShooter()
                                        .until(() -> !commandFactory.shouldShoot()),
                                waitUntil(commandFactory::shouldShoot)
                        )
                ));

        setPassingMode.onTrue(runOnce(() -> {
            shootingIntoHub = false;
            aimingAtHub.set(false);
            passing.set(true);
        })).and(automaticPassingOverride);
        setShootingMode.onTrue(runOnce(() -> {
            shootingIntoHub = true;
            aimingAtHub.set(true);
            passing.set(false);
        })).and(automaticPassingOverride);

        // while shooting and not intaking fuel, use the intake to aid in feeding
        shootTrigger.and(intakeTrigger.negate()).whileTrue(intake.feed());
    }

    private void configureTestBindings() {
        // Tuning Buttons:
        // "A" button toggles tuning mode on and off
        // POV UP/DOWN moves the hood up and down
        // POV LEFT/RIGHT moves the turret left and right
        // X/Y increases and decreases the shooters velocity

        final double TUNING_HOOD_ANGLE_CHANGE = Units.degreesToRadians(0.5);
        final double TUNING_SHOOTER_VELOCITY_CHANGE = Units.rotationsPerMinuteToRadiansPerSecond(50.0);

        final double[] tuningHoodAngle = new double[]{shooter.getConstants().minimumHoodPosition()};
        final double[] tuningShooterVelocity = new double[]{0.0};

        testController.rightBumper().whileTrue(Commands.parallel(
                // TO CHANGE TARGET: Change both the boolean in calibrate shooter and the definition of goalPosition to swap between hub and passing,
                //                      and to choose which passing corner modify CommandFactory.getShooterTarget()
                commandFactory.calibrateShooter(() -> tuningHoodAngle[0], () -> tuningShooterVelocity[0], () -> shootingIntoHub),
                Commands.run(() -> {
                    Translation2d goalPosition = CommandFactory.getShooterTarget(drive.getPose(), CommandFactory.isRedAlliance(), shootingIntoHub);
                    Logger.recordOutput("Tuning/TargetPose", new Pose2d(goalPosition, Rotation2d.kZero));

                    Pose2d robotPose = drive.getPose();
                    Rotation2d turretRotation = Rotation2d.fromRadians(shooter.getTurretPosition());

                    Pose2d turretPose = FireControlSystem.getTurretPose(robotPose, turretRotation);

                    Translation2d delta = goalPosition.minus(turretPose.getTranslation());
                    double distance = delta.getNorm();

                    Logger.recordOutput("Tuning/Distance", distance);

                    Logger.recordOutput("Tuning/TargetHoodAngle", tuningHoodAngle[0]);
                    Logger.recordOutput("Tuning/TargetShooterVelocity", tuningShooterVelocity[0]);
                })
        ));
        testController.rightTrigger().whileTrue(commandFactory.feedIntoShooter());

        testController.povUp().onTrue(runOnce(() -> tuningHoodAngle[0] += TUNING_HOOD_ANGLE_CHANGE));
        testController.povDown().onTrue(runOnce(() -> tuningHoodAngle[0] -= TUNING_HOOD_ANGLE_CHANGE));

        testController.povRight().onTrue(runOnce(() -> tuningShooterVelocity[0] += TUNING_SHOOTER_VELOCITY_CHANGE));
        testController.povLeft().onTrue(runOnce(() -> tuningShooterVelocity[0] -= TUNING_SHOOTER_VELOCITY_CHANGE));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get().command()
                .beforeStarting(() -> ranAuto = true);
    }
}
