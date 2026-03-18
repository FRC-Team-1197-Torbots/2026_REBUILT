package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Utility for computing aim angles with velocity-based lead compensation
 * (shoot while moving). Predicts where the robot will be when the ball
 * arrives and aims at the target from that predicted pose.
 */
public final class ShootingKinematics {

    private ShootingKinematics() {}

    /**
     * Computes the desired turret angle (relative to robot heading, in degrees)
     * to aim at the target, with lead compensation for robot velocity.
     *
     * @param robotPose     Current robot pose (field frame)
     * @param chassisSpeeds Robot chassis speeds (robot frame: vx forward, vy left)
     * @param target        Target position (field frame)
     * @param ballSpeedMps  Estimated ball exit speed (m/s)
     * @param leadScale     Scale factor for lead (1.0 = nominal)
     * @return Desired turret angle in degrees (relative to robot heading)
     */
    public static double getLeadAdjustedAngleDegrees(
            Pose2d robotPose,
            ChassisSpeeds chassisSpeeds,
            Translation2d target,
            double ballSpeedMps,
            double leadScale) {

        Translation2d turretPos = robotPose.getTranslation();
        double distance = turretPos.getDistance(target);
        if (distance < 0.01) {
            return 0.0;
        }

        double speedMagnitude = Math.hypot(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond);

        double timeOfFlight = MathUtil.clamp(
                distance / Math.max(ballSpeedMps, 0.1),
                Constants.ShootingConstants.MinTimeOfFlight,
                Constants.ShootingConstants.MaxTimeOfFlight);

        Pose2d aimPose = robotPose;
        if (speedMagnitude >= Constants.ShootingConstants.VelocityThresholdMps && leadScale > 0) {
            var twist = new edu.wpi.first.math.geometry.Twist2d(
                    chassisSpeeds.vxMetersPerSecond * timeOfFlight * leadScale,
                    chassisSpeeds.vyMetersPerSecond * timeOfFlight * leadScale,
                    chassisSpeeds.omegaRadiansPerSecond * timeOfFlight * leadScale);
            aimPose = robotPose.exp(twist);
        }

        Translation2d aimFrom = aimPose.getTranslation();
        Translation2d delta = target.minus(aimFrom);
        double targetFieldDegrees = delta.getAngle().getDegrees();
        double robotHeadingDegrees = robotPose.getRotation().getDegrees();
        return targetFieldDegrees - robotHeadingDegrees;
    }

    /**
     * Get shooter RPM for a given distance (simple linear model).
     */
    public static double getShooterRPMForDistance(double distanceMeters) {
        return Constants.ShootingConstants.BaseShooterRPM
                + distanceMeters * Constants.ShootingConstants.RpmPerMeter;
    }

    /**
     * Get hood angle (degrees) for a given distance.
     */
    public static double getHoodAngleForDistance(double distanceMeters) {
        return Constants.ShootingConstants.BaseHoodAngleDeg
                + distanceMeters * Constants.ShootingConstants.HoodAnglePerMeter;
    }
}
