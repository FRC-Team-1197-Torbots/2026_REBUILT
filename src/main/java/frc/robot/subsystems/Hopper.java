package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    private final CommandXboxController m_controller;
    
    //motors
    private final SparkFlex hopperMotor1;
    private final SparkFlex hopperMotor2;
    private final TalonFX hopperSplitterMotor;

    //sensors
    private final LaserCan turrent1LaserCan;

    //references
    private final Intake m_intake;
    
    private LaserCan.Measurement m_latestMeasurement;

    public Hopper(CommandXboxController controller, Intake intake) {
        hopperMotor1 = new SparkFlex(HopperConstants.HopperCanId1, SparkLowLevel.MotorType.kBrushless);
        hopperMotor2 = new SparkFlex(HopperConstants.HopperCanId2, SparkLowLevel.MotorType.kBrushless);
        hopperSplitterMotor = new TalonFX(HopperConstants.HooperSplitterCanID, "rio");
        turrent1LaserCan = new LaserCan(Constants.HopperConstants.HooperLaserCANID);

        m_controller = controller;
        m_intake = intake;
    }

    @Override
    public void periodic() {
        super.periodic();
        m_latestMeasurement = turrent1LaserCan.getMeasurement();
        if (m_latestMeasurement != null) {
            SmartDashboard.putNumber("Hopper/LaserDistance", m_latestMeasurement.distance_mm);
            SmartDashboard.putBoolean("Hopper/LaserValid", m_latestMeasurement.status == LaserCan.Status.VALID_MEASUREMENT);
        }
    }

    /** Sets both hopper motors to the same speed. */
    public void setSpeed(double speed) {
        hopperMotor1.set(speed);
        hopperMotor2.set(-speed);
        hopperSplitterMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void feed(double speed) {
        setSpeed(Math.abs(speed));
    }

    public void reverse(double speed) {
        setSpeed(-Math.abs(speed));
    }

    public Command runHopper(double speed) {
        return run(() -> setSpeed(speed));
    }

    /** Runs both hoppers at default feed speed; stops when command ends (e.g. button released). */
    public Command runHopperCommand() {
        return run(() -> {
            // Use cached measurement from periodic()
            // Check for valid measurement and distance threshold
            if (m_latestMeasurement != null && m_latestMeasurement.status == LaserCan.Status.VALID_MEASUREMENT && 
                m_latestMeasurement.distance_mm < HopperConstants.LaserMinDistance) {
                feed(HopperConstants.HopperFeedSpeed);
            } else {
                stop();
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command feedCommand(double speed) {
        return run(() -> feed(speed));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
