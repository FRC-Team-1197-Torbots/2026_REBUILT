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

    public enum ZONE {RED, NEUTRAL, BULE};
    private ZONE myZone;
    private Pigeon2 m_gyro;

    private final String[] limelightNames = {"limelight-front", "limelight-left", "limelight-right"};

    public ZoneDetection(CommandSwerveDrivetrain drivetrain, Pigeon2 gyro) {
        this.drivetrain = drivetrain;
        m_gyro = gyro;
        
        // No need to store NetworkTables, LimelightHelpers handles it by name
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
    }

    private void updatePoseEstimation() {
        for (String limelightName : limelightNames) {
            processLimelight(limelightName);
        }
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
