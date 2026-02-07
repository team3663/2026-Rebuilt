// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.RobotFactory;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

    private final CommandFactory commandFactory;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private boolean shootingAtHub = true;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer(RobotFactory robotFactory) {
        this.drive = robotFactory.createDrive();
        this.feeder = robotFactory.createFeeder();
        this.hopper = robotFactory.createHopper();
        this.intake = robotFactory.createIntake();
        this.shooter = robotFactory.createShooter();
        commandFactory = new CommandFactory(drive, feeder, hopper, intake, shooter
//        , climber
        );
        this.autoPaths = new AutoPaths(drive, feeder, hopper, intake, shooter, commandFactory);


        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive SysId (Quasistatic Forward)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Quasistatic Reverse)",
                drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption(
                "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        autoChooser.addOption(
                "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        autoChooser.addOption("LeftInFrontOfBump5ft-Depot-LeftClimb", autoPaths.leftSide_depot_leftClimb());
        autoChooser.addOption("RightAllianceZone2ft-Outpost-RightClimb", autoPaths.rightSide_outpost_rightClimb());
        autoChooser.addOption("LeftUnderTrench2ft-NeutralZone", autoPaths.leftStarting_neutralZone_middleLine());
        autoChooser.addOption("RightUnderTrench2ft-NeutralZone", autoPaths.rightStarting_neutralZone_middleLine());
        autoChooser.addOption("RightUnderTrench2ft-NeutralZone-LeftClimb", autoPaths.rightStarting_neutralZone_middleLine_leftClimb());
        autoChooser.addOption("LeftUnderTrench2ft-NeutralZone-RightClimb", autoPaths.leftStarting_neutralZone_middleLine_rightClimb());
        autoChooser.addOption("LeftUnderTrench2ft-NeutralZone-UnderTrench-NeutralZone", autoPaths.leftStarting_neutralZone_middleLine_x2());
        autoChooser.addOption("RightUnderTrench2ft-NeutralZone-UnderTrench-NeutralZone", autoPaths.rightStarting_neutralZone_middleLine_x2());
        autoChooser.addOption("LeftInFrontOfBump5ft-Depot-Outpost-RightClimb", autoPaths.leftStarting_depot_outpost_rightClimb());
        autoChooser.addOption("RightAllianceZone2ft-Outpost-Depot-LeftClimb", autoPaths.rightStarting_outpost_depot_leftClimb());

        shooter.setDefaultCommand(shooter.goToDefaultState());

        // Configure the button bindings
        configureButtonBindings();
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

        // Reset gyro to 0° when B button is pressed
        controller.back().onTrue(drive.resetOdometry(() ->
                new Pose2d(
                        drive.getPose().getTranslation(),
                        DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red ?
                                Rotation2d.k180deg :
                                Rotation2d.kZero)));

        controller.start().onTrue(Commands.parallel(shooter.zeroHood()));

        // Intake
        controller.a().onTrue(intake.stop());
        controller.x().whileTrue(intake.intakeAndPivot(6.5, 0.0));

        //Hopper Controls
        controller.b().whileTrue(hopper.withVoltage(3));
        controller.y().onTrue(hopper.stop());

        // Shooter Controls
        controller.leftBumper().onTrue(Commands.runOnce(() -> shootingAtHub = !shootingAtHub));
        controller.rightTrigger().whileTrue(commandFactory.aimShooter(() -> shootingAtHub));

//         controller.x().whileTrue(shooter.runShooter(-8.5));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.get();
    }
}
