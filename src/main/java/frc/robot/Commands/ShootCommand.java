package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ZoneDetection;

public class ShootCommand extends Command {
    private final Shooter m_leftShooter;
    private final Shooter m_rightShooter;
    private final Hopper m_hopper;
    private final ZoneDetection m_zoneDetection;
    private final Timer m_timer;
    private boolean m_hasReachedSpeed;

    public ShootCommand(Shooter leftshooter, Shooter rightshooter, Hopper hopper, ZoneDetection zd) {
        m_leftShooter = leftshooter;
        m_rightShooter = rightshooter;
        m_hopper = hopper;
        m_zoneDetection = zd;
        m_timer = new Timer();
        addRequirements(m_leftShooter, m_rightShooter, m_hopper);
    }

    @Override
    public void initialize() {
        m_zoneDetection.enableZoneDetection(true);
        m_timer.restart();
        m_hasReachedSpeed = false;
        m_leftShooter.setShootingFlag(true);
        m_rightShooter.setShootingFlag(true);
    }

    @Override
    public void execute() {       
        // Tell the shooters to run their active spin logic.
        // They will rely on the target speeds maintained by AimingManager in the background.
        m_leftShooter.Shoot();
        m_rightShooter.Shoot();

        // If we are passing from the neutral or opponent zones, immediately feed the 
        // ball to get rid of it fast, overriding the spool-up delay.
        if (m_zoneDetection.getZone() == ZoneDetection.ZONE.NEUTRAL || m_zoneDetection.isOpponentZone()) {
            m_hopper.feedWithAntiJam(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
        } else {
            // Cross-distance shooting requires both shooters to be fully revved up.
            // Check if shooters have reached speed at least once.
            if (!m_hasReachedSpeed && (m_leftShooter.isAtSpeed() || m_rightShooter.isAtSpeed())) {
                m_hasReachedSpeed = true;
            }

            // Once spooled up or if we timeout after 1 second, feed the balls.
            if (m_hasReachedSpeed || m_timer.hasElapsed(1.0)) {
                m_hopper.feedWithAntiJam(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
            } else {
                m_hopper.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_leftShooter.setShootingFlag(false);
        m_rightShooter.setShootingFlag(false);
        m_leftShooter.Stop();
        m_rightShooter.Stop();
        m_hopper.stop();
    }
}
