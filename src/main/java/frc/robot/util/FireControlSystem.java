package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;

public class FireControlSystem {
    private static final double SPEED_FACTOR = 0.15;

    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE_HUB = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );
    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE_PASS = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );

    static {
        // TODO: get values
        // Hub
        DISTANCE_LOOKUP_TABLE_HUB.put(1.0, new LookupEntry(degreesToRadians(50.0), rotationsPerMinuteToRadiansPerSecond(2500.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(2.0, new LookupEntry(degreesToRadians(40.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(2.5, new LookupEntry(degreesToRadians(32.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.0, new LookupEntry(degreesToRadians(25.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.5, new LookupEntry(degreesToRadians(21.5), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.0, new LookupEntry(degreesToRadians(17.75), rotationsPerMinuteToRadiansPerSecond(3500.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.5, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(3750.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.0, new LookupEntry(degreesToRadians(11.0), rotationsPerMinuteToRadiansPerSecond(4000.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.5, new LookupEntry(degreesToRadians(9.75), rotationsPerMinuteToRadiansPerSecond(4250.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(6.0, new LookupEntry(degreesToRadians(9.25), rotationsPerMinuteToRadiansPerSecond(4250.0)));

        // Passing
        DISTANCE_LOOKUP_TABLE_PASS.put(1.0, new LookupEntry(degreesToRadians(50.0), rotationsPerMinuteToRadiansPerSecond(2500.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(2.0, new LookupEntry(degreesToRadians(40.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(2.5, new LookupEntry(degreesToRadians(32.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(3.0, new LookupEntry(degreesToRadians(25.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(3.5, new LookupEntry(degreesToRadians(21.5), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(4.0, new LookupEntry(degreesToRadians(17.75), rotationsPerMinuteToRadiansPerSecond(3500.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(4.5, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(3750.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(5.0, new LookupEntry(degreesToRadians(11.0), rotationsPerMinuteToRadiansPerSecond(4000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(5.5, new LookupEntry(degreesToRadians(9.75), rotationsPerMinuteToRadiansPerSecond(4250.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(6.0, new LookupEntry(degreesToRadians(9.25), rotationsPerMinuteToRadiansPerSecond(4250.0)));
    }

    public FiringSolution calculate(Pose2d robotPose, ChassisSpeeds fieldOrientedVelocity,
                                    Rotation2d turretRotation,
                                    Translation2d goalPosition, boolean aimAtHub) {
        Pose2d turretPose = getTurretPose(robotPose, turretRotation);

        Translation2d target = goalPosition.minus(leadingOffset(fieldOrientedVelocity));

        Translation2d delta = target.minus(turretPose.getTranslation());

        double distance = delta.getNorm();

        LookupEntry entry;
        if (aimAtHub)
            entry = DISTANCE_LOOKUP_TABLE_HUB.get(distance);
        else
            entry = DISTANCE_LOOKUP_TABLE_PASS.get(distance);

        Rotation2d rotation = delta.getAngle();

        Logger.recordOutput("CommandFactory/TargetTurretPose", new Pose2d(turretPose.getTranslation(), rotation));

        // Add a slight offset when we are shooting at an angle
        return new FiringSolution(rotation.getRadians() - robotPose.getRotation().getRadians(),
                entry.hoodAngle, entry.shooterVelocity);
    }

    public Translation2d leadingOffset(ChassisSpeeds fieldOrientedVelocity) {
        return new Translation2d(
                SPEED_FACTOR * (fieldOrientedVelocity.vxMetersPerSecond + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
                        Math.cos(Math.atan2(Constants.Shooter.TURRET_OFF_CENTER.getY(), Constants.Shooter.TURRET_OFF_CENTER.getX()) - Math.PI / 2)),
                SPEED_FACTOR * (fieldOrientedVelocity.vyMetersPerSecond + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
                        Math.sin(Math.atan2(Constants.Shooter.TURRET_OFF_CENTER.getY(), Constants.Shooter.TURRET_OFF_CENTER.getX()) - Math.PI / 2))
        );
    }

    public Pose2d getTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        Rotation2d turretAngle = robotPose.getRotation().plus(turretRotation);
        Translation2d turretTranslation = robotPose.getTranslation().plus(Constants.Shooter.TURRET_OFF_CENTER);
        Pose2d turretPose = new Pose2d(turretTranslation, turretAngle);
        Logger.recordOutput("CommandFactory/TurretPose", turretPose);
        return turretPose;
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
