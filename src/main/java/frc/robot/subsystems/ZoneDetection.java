package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class ZoneDetection extends SubsystemBase {

    // private final NetworkTable limelightTable; // No longer holding a single
    // table
    private final CommandSwerveDrivetrain drivetrain;

    public enum ZONE {
        RED, NEUTRAL, BLUE
    };

    public ZONE myZone;
    private Pigeon2 m_gyro;

    private final String[] limelightNames = { "limelight-left", "limelight-right" };

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

        // Initialization Check: If we are at 0,0 (likely uninitialized), try to guess
        // based on Alliance
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
        // Update orientation for MegaTag2 for THIS camera (using the SwerveDrivetrain's
        // fused heading or raw yaw)
        // Megatag2 requires CCW-positive heading. Both Pigeon2 yaw and Drivetrain pose
        // rotation are CCW+.
        LimelightHelpers.SetRobotOrientation(name, m_gyro.getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

        // Get the MegaTag2 estimate directly
        // PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
        // //megatag1 in case
        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

        // Basic validation: must have tags and not be an empty pose
        if (mt2.tagCount == 0 || mt2.pose == null) {
            return;
        }

        // --- Standard Deviation Tuning ---
        double xyStdDev;
        // MegaTag2 uses the robot's gyro for rotation, so we tell the pose estimator to
        // NOT trust the vision rotation
        double degStdDev = 9999999.0;

        // Trust multi-tag observations much more
        if (mt2.tagCount >= 2) {
            xyStdDev = 0.3; // Very trustworthy with multiple tags
        }
        // Single tag logic
        else {
            if (mt2.avgTagDist > 4.0) {
                return; // Discard single tags over 4 meters to prevent pose jumps from high ambiguity
            }
            // Scale std dev based on distance squared
            xyStdDev = 0.5 + (mt2.avgTagDist * mt2.avgTagDist / 12.0);
        }

        // Add measurement to drivetrain
        drivetrain.addVisionMeasurement(mt2.pose, mt2.timestampSeconds,
                VecBuilder.fill(xyStdDev, xyStdDev, degStdDev));

        // Just push to dashboard for debugging
        SmartDashboard.putNumber("Vision/" + name + "/TagCount", mt2.tagCount);
        SmartDashboard.putNumber("Vision/" + name + "/AvgDist", mt2.avgTagDist);
    }

    public ZONE getZone() {
        return myZone;
    }
}
