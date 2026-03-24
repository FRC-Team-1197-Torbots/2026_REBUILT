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
    }

    @Override
    public void execute() {
        

        if (m_zoneDetection.getZone() == ZoneDetection.ZONE.NEUTRAL) {
            m_leftShooter.Spin(40.0);
            m_rightShooter.Spin(40.0);
            // Pass functionality: dump balls immediately without waiting for max RPS
            m_hopper.feedWithAntiJam(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
        } else {
            m_leftShooter.Shoot();
            m_rightShooter.Shoot();
            // Shoot functionality: wait until either shooter is at speed (or timeout)
            if (m_leftShooter.isAtSpeed() || m_rightShooter.isAtSpeed() || m_timer.hasElapsed(1.0)) {
                m_hopper.feedWithAntiJam(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
            } else {
                m_hopper.stop();
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_leftShooter.Stop();
        m_rightShooter.Stop();
        m_hopper.stop();
    }
}
