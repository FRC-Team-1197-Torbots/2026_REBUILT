package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ZoneDetection extends SubsystemBase {

    private final NetworkTable limelightTable;
    private final CommandSwerveDrivetrain drivetrain;

    public enum ZONE {RED, NEUTRAL, BULE};
    private ZONE myZone;

    public ZoneDetection(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        limelightTable = NetworkTableInstance.getDefault().getTable("limelight-four");
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
    }

    private void updatePoseEstimation() {
        boolean hasTarget = limelightTable.getEntry("tv").getDouble(0) == 1;

        if (!hasTarget) {
            return;
        }

        double[] botpose = limelightTable.getEntry("botpose_wpiblue").getDoubleArray(new double[0]);

        // get position
        double x = botpose[0];
        double y = botpose[1];

        // get rotation
        Rotation2d rotation = Rotation2d.fromDegrees(botpose[5]);

        Pose2d visionPose = new Pose2d(x, y, rotation);

        double tl = botpose[6];
        double cl = limelightTable.getEntry("cl").getDouble(0);

        double totalLatencySeconds = (tl + cl) / 1000.0;
        double timestamp = Timer.getFPGATimestamp() - totalLatencySeconds;

        double avgDistToTag = botpose[9];
        double xyStdDev = 0.5;
        double degStdDev = 10.0;

        if(avgDistToTag > 4.0) {
            xyStdDev = 3.0;
        } else if(avgDistToTag > 2.0) {
            xyStdDev = 1.0;
        } else {
            xyStdDev = 0.1;
        }

        drivetrain.addVisionMeasurement(visionPose, timestamp,
            VecBuilder.fill(xyStdDev, xyStdDev, Units.degreesToRadians(degStdDev)));

        SmartDashboard.putNumber("LM-X", x);
        SmartDashboard.putNumber("LM-Y", y);
    }

    public ZONE getZone() {
        return myZone;
    }
}
