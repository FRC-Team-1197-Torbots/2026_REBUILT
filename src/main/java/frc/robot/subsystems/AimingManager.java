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
    private final Turret leftTurret;
    // private final Turret rightTurret;
    private final Hood leftHood;
    private final Hood rightHood;
    private final Shooter leftShooter;
    private final Shooter rightShooter;

    // Use WPILib's interpolation map for ball trajectory tuning
    private final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    public AimingManager(CommandSwerveDrivetrain drivetrain, ZoneDetection zoneDetection,
            Turret leftTurret, Hood leftHood, Hood rightHood, Shooter leftShooter, Shooter rightShooter) {
        this.drivetrain = drivetrain;
        this.zoneDetection = zoneDetection;
        this.leftTurret = leftTurret;
        this.leftHood = leftHood;
        this.rightHood = rightHood;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;

        // Trajectory Data: Distance (m) -> Hood Angle (deg)
        // TODO: Tune these placeholder values on the field!
        hoodMap.put(1.78, 0.0);
        hoodMap.put(1.97, 0.0);
        hoodMap.put(2.76, 3.0);
        hoodMap.put(2.58, 3.0);
        hoodMap.put(3.1, 7.0);
        hoodMap.put(3.0, 7.0);

        // Trajectory Data: Distance (m) -> Shooter Speed (RPS - 3000 RPM is 50 RPS)
        // TODO: Tune these placeholder values on the field!
        shooterMap.put(1.97, 5000.0/60);
        shooterMap.put(1.78, 5000.0/60);
        shooterMap.put(2.76, 5000.0/60);
        shooterMap.put(2.58, 5000.0/60);
        shooterMap.put(3.1, 5500.0);
        shooterMap.put(3.18, 5500.0);
        shooterMap.put(7.0, 55.0);
    }

    @Override
    public void periodic() {
        Pose2d targetPose = getTargetPose();

        if (targetPose != null) {
            // 1. Get current robot state
            Pose2d currentRobotPose = drivetrain.getState().Pose;
            double robotHeadingDegrees = currentRobotPose.getRotation().getDegrees();

            // 2. Calculate LEFT Turret & Hood & Shooter
            calculateAndApplyAiming(currentRobotPose, robotHeadingDegrees, targetPose,
                    TurretConstants.TurretOffset1, leftTurret, leftHood, leftShooter, "Left");

            // 3. Calculate RIGHT Turret & Hood & Shooter (when uncommented)
            // calculateAndApplyAiming(currentRobotPose, robotHeadingDegrees, targetPose,
            // TurretConstants.TurretOffset2, rightTurret, rightHood, rightShooter, "Right");
        } else {
            // Idle state if no target is valid
            if (leftTurret != null)
                leftTurret.setTargetAngle(0.0);
            if (leftHood != null)
                leftHood.setTargetAngle(0.0);
            if (leftShooter != null)
                leftShooter.runIdle();
        }
    }

    private void calculateAndApplyAiming(Pose2d robotPose, double robotHeading, Pose2d targetPose,
            Translation2d turretOffset, Turret turret, Hood hood, Shooter shooter, String sideName) {
        if (turret == null && hood == null && shooter == null)
            return;

        // Where is the turret actually located on the field based on the robot's
        // center?
        Pose2d turretFieldPose = robotPose.transformBy(
                new edu.wpi.first.math.geometry.Transform2d(turretOffset, new Rotation2d()));

        Translation2d delta = targetPose.getTranslation().minus(turretFieldPose.getTranslation());

        // ------------- YAW (TURRET) MATH -------------
        double targetFieldDegrees = delta.getAngle().getDegrees();
        double targetRelativeDegrees = targetFieldDegrees - robotHeading;

        // Wrap the angle to handle the -180/180 degree boundary sign flip
        targetRelativeDegrees = MathUtil.inputModulus(targetRelativeDegrees, -180.0, 180.0);

        // Optimize the target angle to fit within the valid physical range of the
        // Turret
        double constrainedYaw = MathUtil.clamp(targetRelativeDegrees, TurretConstants.MinAngle,
                TurretConstants.MaxAngle);

        if (turret != null) {
            turret.setTargetAngle(constrainedYaw);
        }

        // ------------- PITCH (HOOD) MATH & SHOOTER SPEED MATH -------------
        double distanceMeters = delta.getNorm();

        // Calculate Hood Pitch using interpolation map
        double calculatedPitch = hoodMap.get(distanceMeters);
        if (hood != null) {
            hood.setTargetAngle(calculatedPitch);
        }

        // Calculate Shooter Speed using interpolation map
        double calculatedRPS = shooterMap.get(distanceMeters);
        if (shooter != null) {
            shooter.Spin(calculatedRPS);
        }

        // Telemetry
        SmartDashboard.putNumber("AimingManager/" + sideName + "/Distance_m", distanceMeters);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/TargetYaw", constrainedYaw);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/TargetPitch", calculatedPitch);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/TargetRPS", calculatedRPS);
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
