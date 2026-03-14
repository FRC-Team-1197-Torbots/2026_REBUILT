package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.ShootingKinematics;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

/**
 * Shoots while the robot is moving. Aims turret with lead-angle compensation,
 * revs shooter by distance, and feeds when ready. Runs in parallel with path
 * following (does not require drivetrain).
 */
public class ShootWhileMoving extends Command {

    private final Turret m_turret;
    private final Shooter m_shooter;
    private final Hood m_hood;
    private final Hopper m_hopper;
    private final Supplier<Pose2d> m_poseSupplier;
    private final Supplier<ChassisSpeeds> m_speedsSupplier;
    private final Supplier<Translation2d> m_targetSupplier;
    private final boolean m_useLeftShooter;

    private double m_lastTargetAngle = 0;

    /**
     * @param turret          Turret subsystem (e.g. right turret)
     * @param shooter         Shooter subsystem (left or right)
     * @param hood            Hood subsystem (optional - can be null)
     * @param hopper          Hopper subsystem
     * @param poseSupplier    Robot pose (e.g. () -> drivetrain.getState().Pose)
     * @param speedsSupplier  Robot speeds (e.g. () -> drivetrain.getState().Speeds)
     * @param targetSupplier  Target position in field frame
     * @param useLeftShooter  true to use LeftSpin, false for RightSpin
     */
    public ShootWhileMoving(
            Turret turret,
            Shooter shooter,
            Hood hood,
            Hopper hopper,
            Supplier<Pose2d> poseSupplier,
            Supplier<ChassisSpeeds> speedsSupplier,
            Supplier<Translation2d> targetSupplier,
            boolean useLeftShooter) {
        m_turret = turret;
        m_shooter = shooter;
        m_hood = hood;
        m_hopper = hopper;
        m_poseSupplier = poseSupplier;
        m_speedsSupplier = speedsSupplier;
        m_targetSupplier = targetSupplier;
        m_useLeftShooter = useLeftShooter;

        addRequirements(turret, shooter, hopper);
        if (hood != null) {
            addRequirements(hood);
        }
    }

    @Override
    public void initialize() {
        m_lastTargetAngle = m_turret.GetCurrentAngle();
    }

    @Override
    public void execute() {
        Pose2d pose = m_poseSupplier.get();
        ChassisSpeeds speeds = m_speedsSupplier.get();
        Translation2d target = m_targetSupplier.get();

        if (target == null) {
            return;
        }

        double distance = pose.getTranslation().getDistance(target);
        double leadAngle = ShootingKinematics.getLeadAdjustedAngleDegrees(
                pose,
                speeds,
                target,
                Constants.ShootingConstants.BallSpeedMps,
                Constants.ShootingConstants.LeadAngleScale);
        m_lastTargetAngle = MathUtil.clamp(leadAngle,
                Constants.TurretConstants.MinAngle,
                Constants.TurretConstants.MaxAngle);

        m_turret.setTargetAngle(m_lastTargetAngle);

        double rpm = ShootingKinematics.getShooterRPMForDistance(distance);
        if (m_useLeftShooter) {
            m_shooter.LeftSpin(rpm);
        } else {
            m_shooter.RightSpin(rpm);
        }

        if (m_hood != null) {
            double hoodAngle = ShootingKinematics.getHoodAngleForDistance(distance);
            m_hood.setTargetAngle(hoodAngle);
        }

        if (m_turret.isOnTarget(m_lastTargetAngle) && m_shooter.isAtSpeed(rpm)) {
            m_hopper.feed(Constants.HopperConstants.HopperFeedSpeed);
        } else {
            m_hopper.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_hopper.stop();
        if (m_useLeftShooter) {
            m_shooter.LeftSpin(0);
        } else {
            m_shooter.RightSpin(0);
        }
    }
}
