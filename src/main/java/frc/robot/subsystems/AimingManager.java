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
    private final Hood leftHood;
    private final Hood rightHood;

    // Linear interpolation map for Hood ticks based on distance in meters.
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();

    // Shoot-on-the-Move Settings
    
 
    private final String shooterTestRpmKey = "Test Rpm";


    public AimingManager(CommandSwerveDrivetrain drivetrain, ZoneDetection zoneDetection,
            Turret leftTurret, Turret righTurret, Shooter leftShooter, Shooter rightShooter,
            Hood leftHood, Hood rightHood) {
        this.drivetrain = drivetrain;
        this.zoneDetection = zoneDetection;
        this.leftturret = leftTurret;
        this.rightturret = righTurret;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.leftHood = leftHood;
        this.rightHood = rightHood;

        // Setup base values for the hood interpolation table (Distance in meters -> Hood Ticks 0-10)
        // These are placeholder values that you will need to tune on the field!
        hoodMap.put(1.0, 0.0);
        hoodMap.put(3.0, 4.0);
        hoodMap.put(5.0, 8.0);
        hoodMap.put(7.0, 10.0);

        SmartDashboard.putNumber("ShooterTestSpeed", 0);
    }

    public void setShootOnTheMove(boolean enable) {
        // this.enableShootOnTheMove = enable;
    }   

    @Override
    public void periodic() {
        double debugSpeed = SmartDashboard.getNumber("ShooterTestSpeed", 0);

        if(debugSpeed != 0) {
            leftShooter.setShooterSpeed(debugSpeed);
            rightShooter.setShooterSpeed(debugSpeed);
            
        } else {
            leftShooter.setShooterSpeed(Constants.ShooterConstants.IdleSpeed);
            rightShooter.setShooterSpeed(Constants.ShooterConstants.IdleSpeed);
        }

        // Pose2d baseTargetPose = getTargetPose();

        // if (baseTargetPose != null) {
        //     // 1. Get current robot state
        //     Pose2d currentRobotPose = drivetrain.getState().Pose;

        //     // 2. Calculate LEFT Hood & Shooter
        //     calculateAndApplyAiming(currentRobotPose, leftturret, leftShooter, leftHood, "Left");

        //     // 3. Calculate RIGHT Hood & Shooter
        //     calculateAndApplyAiming(currentRobotPose, rightturret, rightShooter, rightHood, "Right");
        // } 
    }

    private void calculateAndApplyAiming(Pose2d robotPose,
            Turret turret, Shooter shooter, Hood hood, String sideName) {

        if (turret == null && shooter == null)
            return;

        double distanceMeters = turret.getDistanceToTarget();

        // double calculatedRPS = SmartDashboard.getNumber(shooterTestRpmKey, 0) / 60.0;
        double calculatedRPS;
        double calculatedHoodTicks;

        if (zoneDetection != null && zoneDetection.getZone() == ZoneDetection.ZONE.NEUTRAL) {
            calculatedRPS = 2500.0 / 60.0;
            // Assuming passing shot has a fixed hood angle, e.g. 5 ticks. Adjust if needed.
            calculatedHoodTicks = 5.0; 
        } else {
            calculatedRPS = calculateRps(distanceMeters);
            calculatedHoodTicks = hoodMap.get(distanceMeters);
        }
        
        if (shooter != null) {
            shooter.setShooterSpeed(calculatedRPS);
        }

        if (hood != null) {
            hood.setTargetAngle(calculatedHoodTicks);
        }

        // Telemetry
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/Distance_m", distanceMeters);
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/HoodTicks", calculatedHoodTicks);
    }

    private double calculateRps(double d) {
        // https://docs.google.com/spreadsheets/d/12vaU1FRqllZlERNKd85nal3VIQaEh6twuFeA2sOHeNw/edit?pli=1&gid=0#gid=0
        double a = 0.3856;
        double b = 5.5;
        double c = 6.0;    
        return a * d * d + b * d + c;
    }

    /**
     * Determines which Pose to aim at based on the Alliance color and
     * ZoneDetection.
     */
    private Pose2d getTargetPose() {
        if (zoneDetection == null) return null;
        
        var alliance = zoneDetection.getAlliance();
        if (alliance.isEmpty()) return null;

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
