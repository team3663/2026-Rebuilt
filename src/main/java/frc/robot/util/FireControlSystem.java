package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.rotationsPerMinuteToRadiansPerSecond;

public class FireControlSystem {
    private static final InterpolatingTreeMap<Double, LookupEntry> DISTANCE_LOOKUP_TABLE = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            LookupEntry::interpolate
    );

    private static final double SHOOTER_VELOCITY_OFFSET = rotationsPerMinuteToRadiansPerSecond(500.0);

    private static final double ANGLED_PIVOT_OFFSET = degreesToRadians(1.0);

    static {
        DISTANCE_LOOKUP_TABLE.put(1.0, new LookupEntry(degreesToRadians(50.0), rotationsPerMinuteToRadiansPerSecond(2500.0)));
        DISTANCE_LOOKUP_TABLE.put(2.0, new LookupEntry(degreesToRadians(40.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE.put(2.5, new LookupEntry(degreesToRadians(32.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE.put(3.0, new LookupEntry(degreesToRadians(25.0), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE.put(3.5, new LookupEntry(degreesToRadians(21.5), rotationsPerMinuteToRadiansPerSecond(3250.0)));
        DISTANCE_LOOKUP_TABLE.put(4.0, new LookupEntry(degreesToRadians(17.75), rotationsPerMinuteToRadiansPerSecond(3500.0)));
        DISTANCE_LOOKUP_TABLE.put(4.5, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(3750.0)));
        DISTANCE_LOOKUP_TABLE.put(5.0, new LookupEntry(degreesToRadians(11.0), rotationsPerMinuteToRadiansPerSecond(4000.0)));
        DISTANCE_LOOKUP_TABLE.put(5.5, new LookupEntry(degreesToRadians(9.75), rotationsPerMinuteToRadiansPerSecond(4250.0)));
        DISTANCE_LOOKUP_TABLE.put(6.0, new LookupEntry(degreesToRadians(9.25), rotationsPerMinuteToRadiansPerSecond(4250.0)));
    }

    public FiringSolution calculate(Pose2d currentPose, ChassisSpeeds currentVelocity, Translation2d goalPosition, double targetHeight) {
        ChassisSpeeds fieldOrientedVelocity =
                ChassisSpeeds.fromRobotRelativeSpeeds(currentVelocity, currentPose.getRotation());

        Translation2d delta = goalPosition.minus(currentPose.getTranslation())
                .minus(new Translation2d(
                        0.15 * fieldOrientedVelocity.vxMetersPerSecond,
                        0.15 * fieldOrientedVelocity.vyMetersPerSecond
                ));

        double distance = delta.getNorm();

        LookupEntry entry = DISTANCE_LOOKUP_TABLE.get(distance + targetHeight / 2.0);

        Rotation2d rotation = delta.getAngle();

        // Add a slight offset when we are shooting at an angle
        double angleOffset = ANGLED_PIVOT_OFFSET * Math.abs(rotation.getSin());

        return new FiringSolution(rotation.getRadians(),
                entry.pivotAngle + angleOffset,
                entry.shooterVelocity + SHOOTER_VELOCITY_OFFSET);
    }

    private record LookupEntry(double pivotAngle, double shooterVelocity) {
        public static LookupEntry interpolate(LookupEntry start, LookupEntry end, double t) {
            return new LookupEntry(
                    MathUtil.interpolate(start.pivotAngle, end.pivotAngle, t),
                    MathUtil.interpolate(start.shooterVelocity, end.shooterVelocity, t)
            );
        }
    }
}
