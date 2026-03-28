package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;

/**
 * The AimingManager is responsible for taking the Robot's current drivetrain
 * pose
 * and the Field Zone to calculate exactly where the Turret (Yaw) and Hood
 * (Pitch)
 * should be aiming.
 * 
 * By pulling this out of Turret.java and Hood.java, those subsystems can remain
 * "dumb" motor controllers, while this class handles all the complex trig and
 * targeting.
 */
public class AimingManager extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final ZoneDetection zoneDetection;

    // References to the hardware subsystems
    private final Turret leftturret;
    private final Turret rightturret;
    private final Shooter leftShooter;
    private final Shooter rightShooter;

    // Use WPILib's interpolation map for ball trajectory tuning
    public InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    // Shoot-on-the-Move Settings
    public boolean enableShootOnTheMove = false;
    private final double AVERAGE_PIECE_SPEED_MPS = 10.0; // Needs tuning
    private final String shooterTestRpmKey = "Test Rpm";


    public AimingManager(CommandSwerveDrivetrain drivetrain, ZoneDetection zoneDetection,
            Turret leftTurret, Turret righTurret, Shooter leftShooter, Shooter rightShooter) {
        this.drivetrain = drivetrain;
        this.zoneDetection = zoneDetection;
        this.leftturret = leftTurret;
        this.rightturret = righTurret;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;

        // Trajectory Data: Distance (m) -> Hood Angle (deg)
        // TODO: Tune these placeholder values on the field!
        hoodMap.put(1.78, 0.0);
        hoodMap.put(1.97, 0.0);
        hoodMap.put(2.76, 0.0);
        hoodMap.put(2.58, 0.0);

        // Trajectory Data: Distance (m) -> Shooter Speed (RPS - 3000 RPM is 50 RPS)
        // TODO: Tune these placeholder values on the field!
        shooterMap.put(1.5, 1700.0/60);
        shooterMap.put(1.9, 1900.0/60);
        shooterMap.put(2.4, 2500.0/60);
        shooterMap.put(4.5, 3700.0/60);
        SmartDashboard.putNumber(shooterTestRpmKey, 0.0);
    }

    public void setShootOnTheMove(boolean enable) {
        this.enableShootOnTheMove = enable;
    }

    private Pose2d applyShootOnTheMove(Pose2d robotPose, Pose2d targetPose) {
        if (!enableShootOnTheMove || targetPose == null) {
            return targetPose;
        }

        edu.wpi.first.math.kinematics.ChassisSpeeds speeds = drivetrain.getState().Speeds;

        double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        double timeOfFlight = distance / AVERAGE_PIECE_SPEED_MPS;

        double offsetX = speeds.vxMetersPerSecond * timeOfFlight;
        double offsetY = speeds.vyMetersPerSecond * timeOfFlight;

        return new Pose2d(
                targetPose.getX() - offsetX,
                targetPose.getY() - offsetY,
                targetPose.getRotation()
        );
    }

    // Reusable instance prevents GC spikes from 50 objects/sec
    private final Pose2d virtualTargetPose = new Pose2d();

    @Override
    public void periodic() {
        Pose2d baseTargetPose = getTargetPose();

        if (baseTargetPose != null) {
            // 1. Get current robot state
            Pose2d currentRobotPose = drivetrain.getState().Pose;
            
            // Generate Virtual Target
            // (Using the pre-allocated virtualTargetPose instance)

            // 2. Calculate LEFT Hood & Shooter
            calculateAndApplyAiming(currentRobotPose, virtualTargetPose,
                    TurretConstants.TurretOffset2, leftturret, leftShooter, "Left");

            // 3. Calculate RIGHT Hood & Shooter
            calculateAndApplyAiming(currentRobotPose, virtualTargetPose,
                    TurretConstants.TurretOffset1, rightturret, rightShooter, "Right");
        } 
    }

    private void calculateAndApplyAiming(Pose2d robotPose, Pose2d targetPose,
            Translation2d turretOffset, Turret turret, Shooter shooter, String sideName) {

        if (turret == null && shooter == null)
            return;

        // Calculate Shooter Speed using interpolation map or override for passing
        

        // double calculatedRPS = SmartDashboard.getNumber(shooterTestRpmKey, 0) / 60.0;
        double calculatedRPS;
        if (zoneDetection != null && zoneDetection.getZone() == ZoneDetection.ZONE.NEUTRAL) {
            calculatedRPS = 2500.0 / 60.0;
        } else {
            calculatedRPS = calculateRps(turret.getDistanceToTarget());
        }
        
        if (shooter != null) {
            shooter.setShooterSpeed(calculatedRPS);
        }

        // Telemetry
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/Distance_m", turret.getDistanceToTarget());
    }

    private double calculateRps(double d) {
        // https://docs.google.com/spreadsheets/d/12vaU1FRqllZlERNKd85nal3VIQaEh6twuFeA2sOHeNw/edit?pli=1&gid=0#gid=0
        double a = 1.78;
        double b = -1.77;
        double c = 21.9;    
        return a * d * d + b * d + c;
    }

    /**
     * Determines which Pose to aim at based on the Alliance color and
     * ZoneDetection.
     */
    private Pose2d getTargetPose() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isEmpty() || zoneDetection == null)
            return null;

        var color = alliance.get();
        var zone = zoneDetection.getZone();
        double yPos = drivetrain.getState().Pose.getY();

        if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
            if (zone == ZoneDetection.ZONE.BLUE)
                return Constants.FieldConstants.BlueTargetPose;
            if (zone == ZoneDetection.ZONE.NEUTRAL) {
                return (yPos < Constants.FieldConstants.FieldWidth / 2.0)
                        ? Constants.FieldConstants.BluePassingCornerRight
                        : Constants.FieldConstants.BluePassingCornerLeft;
            }
        } else if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            if (zone == ZoneDetection.ZONE.RED)
                return Constants.FieldConstants.RedTargetPose;
            if (zone == ZoneDetection.ZONE.NEUTRAL) {
                return (yPos < Constants.FieldConstants.FieldWidth / 2.0)
                        ? Constants.FieldConstants.RedPassingCornerRight
                        : Constants.FieldConstants.RedPassingCornerLeft;
            }
        }
        return null; // Return null if in an enemy zone or unknown
    }
}
