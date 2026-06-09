package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    public record Constants(double minimumTurretPosition, double maximumTurretPosition) {}
}
