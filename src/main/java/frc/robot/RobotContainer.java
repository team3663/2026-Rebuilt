// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.*;
import frc.robot.subsystems.feeder.C2026FeederIO;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.hopper.C2026HopperIO;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.SimHopperIO;
import frc.robot.subsystems.intake.C2026IntakeIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.shooter.C2026ShooterIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.SimShooterIO;
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

    private final CommandFactory commandFactory;

    // Controller
    private final CommandXboxController controller = new CommandXboxController(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> autoChooser;

    private boolean shootingAtHub = true;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                // a CANcoder
                drive = new Drive(
                        new GyroIOPigeon2(),
                        new ModuleIOTalonFX(TunerConstants.FrontLeft),
                        new ModuleIOTalonFX(TunerConstants.FrontRight),
                        new ModuleIOTalonFX(TunerConstants.BackLeft),
                        new ModuleIOTalonFX(TunerConstants.BackRight));

                feeder = new Feeder(new C2026FeederIO(
                        new TalonFX(14),
                        new TalonFX(15),
                        new CANrange(1))
                );
                hopper = new Hopper(new C2026HopperIO(new TalonFX(10)));

                intake = new Intake(new C2026IntakeIO(
                        new TalonFX(11),
                        new TalonFX(12),
                        new TalonFX(13)
                ));

                shooter = new Shooter(new C2026ShooterIO(
                        new TalonFX(16),
                        new TalonFX(17),
                        new TalonFX(18),
                        new TalonFX(19),
                        new CANcoder(7),
                        new CANcoder(8)
                ));


                // The ModuleIOTalonFXS implementation provides an example implementation for
                // TalonFXS controller connected to a CANdi with a PWM encoder. The
                // implementations
                // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
                // swerve
                // template) can be freely intermixed to support alternative hardware
                // arrangements.
                // Please see the AdvantageKit template documentation for more information:
                // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
                //
                // drive =
                // new Drive(
                // new GyroIOPigeon2(),
                // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
                // new ModuleIOTalonFXS(TunerConstants.FrontRight),
                // new ModuleIOTalonFXS(TunerConstants.BackLeft),
                // new ModuleIOTalonFXS(TunerConstants.BackRight));
                break;

            case SIM:
                // Sim robot, instantiate physics sim IO implementations
                drive = new Drive(
                        new GyroIO() {
                        },
                        new ModuleIOSim(TunerConstants.FrontLeft),
                        new ModuleIOSim(TunerConstants.FrontRight),
                        new ModuleIOSim(TunerConstants.BackLeft),
                        new ModuleIOSim(TunerConstants.BackRight));
                feeder = new Feeder(new FeederIO() {
                });
                hopper = new Hopper(new SimHopperIO());
                intake = new Intake(new IntakeIO() {
                });
                shooter = new Shooter(new SimShooterIO());
                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                },
                                new ModuleIO() {
                                });
                feeder = new Feeder(new FeederIO() {
                });
                hopper = new Hopper(new HopperIO() {
                });
                intake = new Intake(new IntakeIO() {
                });
                shooter = new Shooter(new ShooterIO() {
                });
                break;
        }

        commandFactory = new CommandFactory(drive, feeder, hopper, intake, shooter
//        , climber
        );

        // Set up auto routines
        autoChooser = new LoggedDashboardChooser<>("Auto Choices", new SendableChooser<>());

        // Set up SysId routines
        autoChooser.addOption(
                "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        autoChooser.addOption(
                "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
        drive.setDefaultCommand(
                DriveCommands.joystickDrive(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX()));

//        // Lock to 0° when A button is held
//        controller
//                .a()
//                .whileTrue(
//                        DriveCommands.joystickDriveAtAngle(
//                                drive,
//                                () -> -controller.getLeftY(),
//                                () -> -controller.getLeftX(),
//                                () -> Rotation2d.kZero));
//
//        // Switch to X pattern when X button is pressed
//        controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
//
//        // Reset gyro to 0° when B button is pressed
//        controller
//                .b()
//                .onTrue(
//                        Commands.runOnce(
//                                        () ->
//                                                drive.setPose(
//                                                        new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
//                                        drive)
//                                .ignoringDisable(true));
//
         // Zeroing
//        controller.back().onTrue(drive.resetFieldOriented());
        controller.start().onTrue(Commands.parallel(shooter.zeroHood()));

        // Intake
        controller.a().onTrue(intake.stopIntake());
        controller.x().whileTrue(intake.intakeWithVoltage(3.0));

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
