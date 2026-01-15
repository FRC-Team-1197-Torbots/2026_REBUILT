package frc.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTurnToAngle extends Command {

    private double targetAngle;
    private Turret m_turret;
    private final double err = 0.05f;

    public TurretTurnToAngle(double angle, Turret turret) {
        targetAngle = angle;
        m_turret = turret;
    }

    @Override
    public void execute() {
        double currentdelta = targetAngle - m_turret.GetCurrentAngle();

        double power = currentdelta * 0.01d;

        m_turret.setPower((float)power);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.StopTurret();
    }

    @Override
    public boolean isFinished() {
        if (m_turret.GetCurrentAngle() >= targetAngle * (1 - err)
                && m_turret.GetCurrentAngle() <= targetAngle * (1 + err)) {
            return true;
        } else {
            return false;
        }
    }
}
