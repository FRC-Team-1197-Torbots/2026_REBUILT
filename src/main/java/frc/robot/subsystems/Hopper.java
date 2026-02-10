package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    private final SparkFlex hopperMotor;
    
    private double hopperSpeed = 0.0;
    
    public Hopper() {
        hopperMotor = new SparkFlex(HopperConstants.HopperCanId, SparkLowLevel.MotorType.kBrushless);
    }
    
    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Hopper Set Speed", hopperSpeed);
        SmartDashboard.putNumber("Hopper Actual Output", hopperMotor.getAppliedOutput());
    }
    
    public void setSpeed(double speed) {
        hopperSpeed = speed;
        hopperMotor.set(speed);
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
    
    public Command feedCommand(double speed) {
        return run(() -> feed(speed));
    }
    
    public Command stopCommand() {
        return runOnce(() -> stop());
    }
}
