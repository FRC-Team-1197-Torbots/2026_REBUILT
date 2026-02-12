package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class ZoneDetection extends SubsystemBase {

    // private final NetworkTable limelightTable; // No longer holding a single table
    private final CommandSwerveDrivetrain drivetrain;

    public enum ZONE {RED, NEUTRAL, BLUE};
    public ZONE myZone;
    private Pigeon2 m_gyro;

    private final String[] limelightNames = {"limelight-front", "limelight-left", "limelight-right"};

    public ZoneDetection(CommandSwerveDrivetrain drivetrain, Pigeon2 gyro) {
        this.drivetrain = drivetrain;
        m_gyro = gyro;
        
        // No need to store NetworkTables, LimelightHelpers handles it by name
        
        // Default to Blue if unknown
        myZone = ZONE.BLUE;
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
        updateZone();
    }

    private void updatePoseEstimation() {
        for (String limelightName : limelightNames) {
            processLimelight(limelightName);
        }
    }

    private void updateZone() {
        // Get current robot X position (Blue Alliance Origin)
        double botX = drivetrain.getState().Pose.getX();

        // Initialization Check: If we are at 0,0 (likely uninitialized), try to guess based on Alliance
        if (botX == 0.0 && drivetrain.getState().Pose.getY() == 0.0) {
            var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                    myZone = ZONE.RED;
                } else {
                    myZone = ZONE.BLUE;
                }
            }
            // Don't run the coordinate check if we are uninitialized
            SmartDashboard.putString("Zone", myZone.toString());
            return;
        }
        
        double blueLine = frc.robot.Constants.FieldConstants.BlueAllianceLineX;
        double redLine = frc.robot.Constants.FieldConstants.RedAllianceLineX;

        if (botX < blueLine) {
            myZone = ZONE.BLUE;
        } else if (botX > redLine) {
            myZone = ZONE.RED;
        } else {
            myZone = ZONE.NEUTRAL;
        }
        
        SmartDashboard.putString("Zone", myZone.toString());
    }
    private void processLimelight(String name) {
        // Update orientation for MegaTag2 for THIS camera
        LimelightHelpers.SetRobotOrientation(name, m_gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
        
        // Get the MegaTag2 estimate directly
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // Basic validation: must have tags and not be an empty pose
        if (mt2.tagCount == 0 || mt2.pose == null) {
            return;
        }

        // --- Standard Deviation Tuning ---
        double xyStdDev = 0.5;
        double degStdDev = 10.0;

        // Trust multi-tag observations much more
        if (mt2.tagCount >= 2) {
            xyStdDev = 0.3;
            degStdDev = 1.0; 
        } 
        // Single tag logic
        else {
            if (mt2.avgTagDist > 5.0) {
                xyStdDev = 5.0; // Very untrustworthy at range
            } else if (mt2.avgTagDist > 3.0) {
                xyStdDev = 1.0;
            } else {
                xyStdDev = 0.5;
            }
        }

        // Add measurement to drivetrain
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
            VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(degStdDev)));

        // Just push to dashboard for debugging (maybe just the front one or average?)
        SmartDashboard.putNumber("Vision/" + name + "/TagCount", mt2.tagCount);
        SmartDashboard.putNumber("Vision/" + name + "/AvgDist", mt2.avgTagDist);
    }

    public ZONE getZone() {
        return myZone;
    }
}
