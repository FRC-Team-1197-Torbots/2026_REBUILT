package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
    private TalonFX hoodMain;
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    private final NeutralOut stophoodrequest = new NeutralOut();

    private double PIDTolerance = 0.3d;
    private double Target = -1;

    public enum HOOD_SIDE {
        RIGHT, LEFT
    };

    protected HOOD_SIDE m_side;

    //max rotations 10
    public Hood(int hoodCanId, HOOD_SIDE side) {
        m_side = side;
        hoodMain = new TalonFX(hoodCanId);

        var hoodConfig = new TalonFXConfiguration();
        hoodConfig.Slot0.kP = Constants.HoodConstants.kP;
        hoodConfig.Slot0.kI = Constants.HoodConstants.kI;
        hoodConfig.Slot0.kD = Constants.HoodConstants.kD;

        hoodMain.setNeutralMode(NeutralModeValue.Brake);
        hoodMain.getConfigurator().apply(hoodConfig);
        hoodMain.setPosition(0);      
          
    }

    public void periodic() {
        super.periodic();
                
        double delta = Math.abs(Target - hoodMain.getPosition().getValueAsDouble());

        if(delta <= PIDTolerance) {
            hoodMain.setControl(stophoodrequest);
        }        
    }

    public Command ManualHoodUp() {
        return Commands.runOnce(() -> hoodMain.set(0.3));
    }

    public Command ManualHoodDown() {
        return Commands.runOnce(() -> hoodMain.set(-0.3));
    }

    public Command ManualHoodStop() {
        return Commands.runOnce(() -> hoodMain.set(0));
    }

    //provides ticks in degrees
    public void setTargetAngle(double ticks) {
        Target = ticks;
        //double ticks = ConvertDegreesToTicks(targetDegrees);
        //SmartDashboard.putNumber("Hood " + m_side.name() + "/Requested Ticks", ticks);
        hoodMain.setControl(m_request.withPosition(ticks).withSlot(0));
    }

    public void setPower(double power) {
        hoodMain.set(power);
    }
}
