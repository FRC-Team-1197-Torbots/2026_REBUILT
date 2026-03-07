package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private TalonFX hoodMain;
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Hood(int hoodCanId) {
        hoodMain = new TalonFX(hoodCanId);

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = Constants.HoodConstants.kP;
        hoodConfig.Slot0.kI = Constants.HoodConstants.kI;
        hoodConfig.Slot0.kD = Constants.HoodConstants.kD;

        hoodMain.getConfigurator().apply(hoodConfig);
        hoodMain.setPosition(0);

        SmartDashboard.putNumber("Hood/Hood Angle", 0);
    }

    public Command ManualHoodUp() {
        return Commands.runOnce(() -> hoodMain.set(0.1));
    }

    public Command ManualHoodDown() {
        return Commands.runOnce(() -> hoodMain.set(-0.1));
    }

    public Command ManualHoodStop() {
        return Commands.runOnce(() -> hoodMain.set(0));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Hood/Current Angle", ConvertTicksToAngle(hoodMain.getPosition().getValueAsDouble()));
    }

    public void setTargetAngle(double targetDegrees) {
        double ticks = ConvertDegreesToTicks(targetDegrees);
        SmartDashboard.putNumber("Hood/Requested Ticks", ticks);
        hoodMain.setControl(m_request.withPosition(ticks));
    }

    public void setPower(double power) {
        hoodMain.set(power);
    }

    private double ConvertTicksToAngle(double ticks) {
        return ticks * HoodConstants.DegreesPerRotation;
    }

    private double ConvertDegreesToTicks(double angle) {
        return angle / HoodConstants.DegreesPerRotation;
    }
}
