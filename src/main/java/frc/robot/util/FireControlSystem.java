package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;

public class FireControlSystem {
    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE_HUB = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );
    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE_PASS = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );
    private static final InterpolatingTreeMap<Double, Double> DISTANCE_LOOKUP_TABLE_LEAD = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            Interpolator.forDouble()
    );

    static {
        // Hub
        DISTANCE_LOOKUP_TABLE_HUB.put(2.0, new LookupEntry(degreesToRadians(1.0), rotationsPerMinuteToRadiansPerSecond(2050.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(2.5, new LookupEntry(degreesToRadians(3.0), rotationsPerMinuteToRadiansPerSecond(2150.0)));
        // Look at 3.0 and 3.5 distances
        DISTANCE_LOOKUP_TABLE_HUB.put(3.0, new LookupEntry(degreesToRadians(5.0), rotationsPerMinuteToRadiansPerSecond(2300.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.25, new LookupEntry(degreesToRadians(5.0), rotationsPerMinuteToRadiansPerSecond(2400.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.5, new LookupEntry(degreesToRadians(5.5), rotationsPerMinuteToRadiansPerSecond(2450.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.75, new LookupEntry(degreesToRadians(5.5), rotationsPerMinuteToRadiansPerSecond(2450.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.0, new LookupEntry(degreesToRadians(6.5), rotationsPerMinuteToRadiansPerSecond(2500.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.5, new LookupEntry(degreesToRadians(7.0), rotationsPerMinuteToRadiansPerSecond(2625.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.0, new LookupEntry(degreesToRadians(7.0), rotationsPerMinuteToRadiansPerSecond(2700.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.5, new LookupEntry(degreesToRadians(7.0), rotationsPerMinuteToRadiansPerSecond(2800.0)));

        // Passing
        DISTANCE_LOOKUP_TABLE_PASS.put(4.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(1950.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(5.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(2050.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(6.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(2200.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(7.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(2400.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(8.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(2600.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(9.0, new LookupEntry(degreesToRadians(16.0), rotationsPerMinuteToRadiansPerSecond(2850.0)));

        DISTANCE_LOOKUP_TABLE_PASS.put(10.0, new LookupEntry(degreesToRadians(17.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(11.0, new LookupEntry(degreesToRadians(17.0), rotationsPerMinuteToRadiansPerSecond(3200.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(15.0, new LookupEntry(degreesToRadians(18.0), rotationsPerMinuteToRadiansPerSecond(3900.0)));


        // Leading
        DISTANCE_LOOKUP_TABLE_LEAD.put(2.0, 1.03);
        DISTANCE_LOOKUP_TABLE_LEAD.put(2.5, 1.12);
        DISTANCE_LOOKUP_TABLE_LEAD.put(3.0, 1.15);
        DISTANCE_LOOKUP_TABLE_LEAD.put(3.5, 1.19);
        DISTANCE_LOOKUP_TABLE_LEAD.put(4.0, 1.25);
        DISTANCE_LOOKUP_TABLE_LEAD.put(4.5, 1.13);
        DISTANCE_LOOKUP_TABLE_LEAD.put(5.0, 1.35);
    }

    private final LoggedNetworkNumber shooterVelocityTrimEntry = new LoggedNetworkNumber("Shooter Velocity Trim", 0.0);
    private final LoggedNetworkNumber turretAngleTrimEntry = new LoggedNetworkNumber("Turret Angle Trim", 0.0);


    public FiringSolution calculate(Pose2d robotPose, ChassisSpeeds fieldOrientedVelocity,
                                    Rotation2d turretRotation,
                                    Translation2d goalPosition, boolean aimAtHub) {
        Pose2d turretPose = getTurretPose(robotPose, turretRotation);

        double originalDistance = goalPosition.minus(turretPose.getTranslation()).getNorm();
        Translation2d leadTarget = goalPosition.minus(getLeadingOffset(fieldOrientedVelocity, originalDistance));
        Logger.recordOutput("CommandFactory/LeadGoalPose", new Pose2d(leadTarget, Rotation2d.kZero));

        Translation2d delta = leadTarget.minus(turretPose.getTranslation());

        double distance = delta.getNorm();

        LookupEntry entry;
        if (aimAtHub)
            entry = DISTANCE_LOOKUP_TABLE_HUB.get(distance);
        else
            entry = DISTANCE_LOOKUP_TABLE_PASS.get(distance);

        Rotation2d rotation = delta.getAngle();

        Logger.recordOutput("CommandFactory/TargetTurretPose", new Pose2d(turretPose.getTranslation(), rotation));

        double shooterVelocityTrim = Units.rotationsPerMinuteToRadiansPerSecond(shooterVelocityTrimEntry.getAsDouble());
        double turretAngleTrim = Units.degreesToRadians(turretAngleTrimEntry.getAsDouble());

        // Add a slight offset when we are shooting at an angle
        return new FiringSolution((rotation.getRadians() - robotPose.getRotation().getRadians() + turretAngleTrim),
                entry.hoodAngle, entry.shooterVelocity + shooterVelocityTrim);
    }

    public static Pose2d getTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        Pose2d turretPose = robotPose.plus(
                new Transform2d(
                        Constants.Shooter.TURRET_OFF_CENTER,
                        turretRotation));
        Logger.recordOutput("CommandFactory/TurretPose", turretPose);
        return turretPose;
    }

    public Translation2d getLeadingOffset(ChassisSpeeds fieldOrientedVelocity, double distance) {
        double speedFactor = DISTANCE_LOOKUP_TABLE_LEAD.get(distance);
        return new Translation2d(
                speedFactor * (fieldOrientedVelocity.vxMetersPerSecond
//                        + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
//                        Math.cos(Math.atan2(Constants.Shooter.TURRET_OFF_CENTER.getY(), Constants.Shooter.TURRET_OFF_CENTER.getX()) - Math.PI / 2)
                ),
                speedFactor * (fieldOrientedVelocity.vyMetersPerSecond
//                        + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
//                        Math.sin(Math.atan2(Constants.Shooter.TURRET_OFF_CENTER.getY(), Constants.Shooter.TURRET_OFF_CENTER.getX()) - Math.PI / 2)
                )
        );
    }

    private record LookupEntry(double hoodAngle, double shooterVelocity) {
        public static LookupEntry interpolate(LookupEntry start, LookupEntry end, double t) {
            return new LookupEntry(
                    MathUtil.interpolate(start.hoodAngle, end.hoodAngle, t),
                    MathUtil.interpolate(start.shooterVelocity, end.shooterVelocity, t)
            );
        }
    }
}
