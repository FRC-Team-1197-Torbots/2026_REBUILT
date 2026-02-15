package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public TalonFX climbMotor;
    public CANrange range; 


    public Climber() {
        
    }

    public Command getAlignToClimbCommand() {
        Pose2d targetClimbPose = Constants.ClimberConstants.lowerClimbPosition;

        PathConstraints constraints = new PathConstraints(3.5, 
            3.5, Units.degreesToRadians(540),
            Units.degreesToRadians(360));

        return AutoBuilder.pathfindToPose(targetClimbPose, constraints,0.0f);
    }

}
