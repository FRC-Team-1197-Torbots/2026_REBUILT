package frc.Commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Turret;

public class TurretTurnToAngle extends Command {

    private double targetAngle;
    private Turret m_turret;
    private final double err = 0.05f;
    private final double p = 0.003d;

    public TurretTurnToAngle(double angle, Turret turret) {
        targetAngle = angle;
        m_turret = turret;
    }

    @Override
    public void execute() {
        double currentdelta = targetAngle - m_turret.GetCurrentAngle();

        double dir = currentdelta * p;
        SmartDashboard.putNumber("Turret Power", dir);

        m_turret.setPower((float)dir);
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.setPower(0);
    }

    @Override
    public boolean isFinished() {
        if (m_turret.GetCurrentAngle() >= targetAngle) {
            return true;
        } else {
            return false;
        }
    }
}
