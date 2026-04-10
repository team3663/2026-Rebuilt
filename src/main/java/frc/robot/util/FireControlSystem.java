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
import edu.wpi.first.wpilibj.DriverStation;
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
        DISTANCE_LOOKUP_TABLE_HUB.put(2.0, new LookupEntry(degreesToRadians(0.0), rotationsPerMinuteToRadiansPerSecond(2620.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(2.5, new LookupEntry(degreesToRadians(1.0), rotationsPerMinuteToRadiansPerSecond(2770.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.0, new LookupEntry(degreesToRadians(2.0), rotationsPerMinuteToRadiansPerSecond(2870.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(3.5, new LookupEntry(degreesToRadians(3.0), rotationsPerMinuteToRadiansPerSecond(3175.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.0, new LookupEntry(degreesToRadians(9.0), rotationsPerMinuteToRadiansPerSecond(3175.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(4.5, new LookupEntry(degreesToRadians(9.0), rotationsPerMinuteToRadiansPerSecond(3375.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.0, new LookupEntry(degreesToRadians(9.5), rotationsPerMinuteToRadiansPerSecond(3575.0)));
        DISTANCE_LOOKUP_TABLE_HUB.put(5.5, new LookupEntry(degreesToRadians(10.5), rotationsPerMinuteToRadiansPerSecond(3625.0)));

        // Passing
        DISTANCE_LOOKUP_TABLE_PASS.put(3.0, new LookupEntry(degreesToRadians(11.5), rotationsPerMinuteToRadiansPerSecond(2300.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(4.0, new LookupEntry(degreesToRadians(12.5), rotationsPerMinuteToRadiansPerSecond(2700.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(5.0, new LookupEntry(degreesToRadians(14.5), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(6.0, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(3000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(7.0, new LookupEntry(degreesToRadians(14.0), rotationsPerMinuteToRadiansPerSecond(3050.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(8.0, new LookupEntry(degreesToRadians(15.0), rotationsPerMinuteToRadiansPerSecond(3350.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(9.0, new LookupEntry(degreesToRadians(15.5), rotationsPerMinuteToRadiansPerSecond(4000.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(10.0, new LookupEntry(degreesToRadians(16.0), rotationsPerMinuteToRadiansPerSecond(4500.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(11.0, new LookupEntry(degreesToRadians(16.0), rotationsPerMinuteToRadiansPerSecond(4750.0)));
        DISTANCE_LOOKUP_TABLE_PASS.put(12.0, new LookupEntry(degreesToRadians(16.5), rotationsPerMinuteToRadiansPerSecond(5100.0)));


        // Leading
        DISTANCE_LOOKUP_TABLE_LEAD.put(2.0, 1.0);
        DISTANCE_LOOKUP_TABLE_LEAD.put(2.5, 1.15);
        DISTANCE_LOOKUP_TABLE_LEAD.put(3.0, 1.3);
        DISTANCE_LOOKUP_TABLE_LEAD.put(3.5, 1.0);
        DISTANCE_LOOKUP_TABLE_LEAD.put(4.0, 1.3);
        DISTANCE_LOOKUP_TABLE_LEAD.put(4.5, 1.25);
        DISTANCE_LOOKUP_TABLE_LEAD.put(5.0, 1.35);
    }

    private final LoggedNetworkNumber shooterVelocityTrimEntry = new LoggedNetworkNumber("Shooter Velocity Trim", 0.0);
    private final LoggedNetworkNumber turretAngleTrimEntry = new LoggedNetworkNumber("Turret Angle Trim", 0.0);


    public FiringSolution calculate(Pose2d robotPose, ChassisSpeeds fieldOrientedVelocity,
                                    Rotation2d turretRotation,
                                    Translation2d goalPosition, boolean aimAtHub) {
        Pose2d turretPose = getTurretPose(robotPose, turretRotation);

        Translation2d leadTarget = getLeadedTarget(fieldOrientedVelocity, turretPose.getTranslation(), goalPosition);
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

        var notShootingUnderTrench = true;
        var poseX = turretPose.getX();
        var poseY = turretPose.getY();

        if (!DriverStation.isAutonomous()) {
            if (((poseX >= Constants.BLUE_ALLIANCE_SIDE_TRENCH_X && poseX <= Constants.BLUE_NZ_SIDE_TRENCH_X)
                    || (poseX >= Constants.RED_NZ_SIDE_TRENCH_X && poseX <= Constants.RED_ALLIANCE_SIDE_TRENCH_X))
                    && (poseY <= Constants.RIGHT_TRENCH_LEFT_Y || poseY >= Constants.LEFT_TRENCH_RIGHT_Y))
                notShootingUnderTrench = false;
        }

        Logger.recordOutput("CommandFactory/NotShootingUnderTrench-FCS", notShootingUnderTrench);

        // Add a slight offset when we are shooting at an angle
        return new FiringSolution((rotation.getRadians() - robotPose.getRotation().getRadians() + turretAngleTrim),
                notShootingUnderTrench ?  entry.hoodAngle : 0.0, entry.shooterVelocity + shooterVelocityTrim);
    }

    public static Pose2d getTurretPose(Pose2d robotPose, Rotation2d turretRotation) {
        Pose2d turretPose = robotPose.plus(
                new Transform2d(
                        Constants.Shooter.TURRET_OFF_CENTER,
                        turretRotation));
        Logger.recordOutput("CommandFactory/TurretPose", turretPose);
        return turretPose;
    }

    public Translation2d getLeadedTarget(ChassisSpeeds fieldOrientedVelocity, Translation2d turretPosition, Translation2d targetPosition) {
        Translation2d leadedTarget = targetPosition;
        double distance;

        for (int i = 0; i < 20; i++) {
            distance = turretPosition.getDistance(leadedTarget);
            leadedTarget = targetPosition.minus(getLeadingOffset(fieldOrientedVelocity, distance));
        }

        return leadedTarget;
    }

    private Translation2d getLeadingOffset(ChassisSpeeds fieldOrientedVelocity, double distance) {
        double speedFactor = DISTANCE_LOOKUP_TABLE_LEAD.get(distance);
        return new Translation2d(
                speedFactor * (fieldOrientedVelocity.vxMetersPerSecond + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
                        Math.cos(Constants.Shooter.TURRET_OFF_CENTER.getAngle().getRadians() - Math.PI / 2)
                ),
                speedFactor * (fieldOrientedVelocity.vyMetersPerSecond + fieldOrientedVelocity.omegaRadiansPerSecond * Constants.Shooter.TURRET_OFF_CENTER.getNorm() *
                        Math.sin(Constants.Shooter.TURRET_OFF_CENTER.getAngle().getRadians() - Math.PI / 2)
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
