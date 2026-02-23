package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HoodConstants;

public class Hood extends SubsystemBase {
    private SparkFlex hood;
    private SparkClosedLoopController hoodController;
    
    private double hoodangle = 0;

    public Hood() {
        hood = new SparkFlex(HoodConstants.HoodCanId, MotorType.kBrushless);
        hoodController = hood.getClosedLoopController();
        
        // Configure PID and Safety
        com.revrobotics.spark.config.SparkFlexConfig config = new com.revrobotics.spark.config.SparkFlexConfig();
        config.closedLoop.pid(HoodConstants.kP, 0.0, 0.0, com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor.kPrimaryEncoder);
        config.closedLoop.outputRange(-1, 1);
        
        // Apply configuration
        hood.configure(config, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, com.revrobotics.spark.SparkBase.PersistMode.kPersistParameters);

        hood.getEncoder().setPosition(0);
        
        SmartDashboard.putNumber("Hood Angle", 0);
    }

    @Override
    public void periodic() {
        hoodangle = SmartDashboard.getNumber("Hood Angle", 0);
        SmartDashboard.putNumber("Actual Hood Angle", hood.getEncoder().getPosition() * HoodConstants.DegreesPerRotation);
    }

    public void GoToAngle() {
        double ticks = ConvertDegreesToTicks(hoodangle);
        SmartDashboard.putNumber("Requested Ticks", ticks);
        hoodController.setSetpoint(ticks, ControlType.kPosition);
    }
    
    private double ConvertTicksToAngle(double ticks) {
         return ticks * HoodConstants.DegreesPerRotation;
    }

    private double ConvertDegreesToTicks(double angle) {
        return angle / HoodConstants.DegreesPerRotation; 
    }
}
