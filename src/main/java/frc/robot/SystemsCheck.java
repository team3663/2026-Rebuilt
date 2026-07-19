package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;

public class SystemsCheck {
    private final Drive drive;
    private final Feeder feeder;
    private final Hopper hopper;
    private final Intake intake;
    private final Shooter shooter;
    private final CommandXboxController controller;

    //TODO get values
    //constants for testing positions
    private final double INTAKE_TEST_POSITION = Units.degreesToRadians(90);
    private final double HOOD_TEST_POSITION = Units.degreesToRadians(10);
    private final double TURRET_TEST_POSITION = Units.degreesToRadians(90);

    private final double TUNNEL_TEST_VOLTAGE = 5;
    private final double TOP_ROLLER_TEST_VOLTAGE = 5;
    private final double HOPPER_TEST_VOLTAGE = 5;
    private final double FEEDER_TEST_VOLTAGE = 5;

    SystemsCheck(Drive drive, Feeder feeder, Hopper hopper, Intake intake, Shooter shooter, CommandXboxController controller) {
        this.drive = drive;
        this.feeder = feeder;
        this.hopper = hopper;
        this.intake = intake;
        this.shooter = shooter;
        this.controller = controller;
    }

    public  Command systemsCheck() {
        return new SequentialCommandGroup(
            zeroAll().until(() -> intake.isZeroed() && shooter.isZeroed()),
            testDrive(),
            testPositions(),
            allSystemsTest());
    }

    //runs the drivetrain forwards, to the left, and rotating
    //runs at 1 foot per second laterally, and wheels at 60rpm for rotation
    public Command testDrive() {
        return new SequentialCommandGroup(
                drive.drivetrainTest(Units.feetToMeters(1), 0, 0).withDeadline(waitSeconds(1).andThen(waitUntil(controller.a()))),
                drive.drivetrainTest(0, Units.feetToMeters(1), 0).withDeadline(waitSeconds(1).andThen(waitUntil(controller.a()))),
                drive.drivetrainTest(0, 0, Units.rotationsPerMinuteToRadiansPerSecond(30)).withDeadline(waitSeconds(1)).andThen(waitUntil(controller.a())),
                drive.drivetrainTest(0, 0, 0).withDeadline(waitUntil(controller.a())));
    }

    //sets the intake and shooter to a specified position to be checked manually
    public Command testPositions() {
        return new SequentialCommandGroup(
                //TODO clean this up
                intake.intakeAndPivot(4, INTAKE_TEST_POSITION)
                        .withDeadline(waitUntil(controller.a().negate())).andThen(waitUntil(controller.a())).andThen(intake.stop()),
                shooter.goTo(HOOD_TEST_POSITION, TURRET_TEST_POSITION, 0, false)
                        .withDeadline(waitUntil(controller.a().negate())).andThen(waitUntil(controller.a())).andThen(shooter.stop()));
    }

    //tests shooting at low speed
    public Command allSystemsTest() {
        return new ParallelCommandGroup(
          intake.deployAndIntake(),
          hopper.withVoltage(TUNNEL_TEST_VOLTAGE, HOPPER_TEST_VOLTAGE, TOP_ROLLER_TEST_VOLTAGE),
          feeder.withVoltage(FEEDER_TEST_VOLTAGE),
          shooter.goTo(Units.degreesToRadians(10), Units.degreesToRadians(90), Units.rotationsPerMinuteToRadiansPerSecond(1000), true)
        );
    }

    //zeros all zeroable subsystems
    public Command zeroAll() {
        return new ParallelCommandGroup(intake.zeroPivot(), shooter.zeroHood());
    }
}