package frc.robot.subsystems;

import java.nio.file.DirectoryStream.Filter;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    // Shoot-on-the-Move Settings

    private final String shooterTestRpmKey = "Test Rpm";

    private LinearFilter filter = LinearFilter.singlePoleIIR(0.7, 0.02);

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

        

        SmartDashboard.putNumber("ShooterTestSpeed", 0);
    }

    public void setShootOnTheMove(boolean enable) {
        // this.enableShootOnTheMove = enable;
    }

    @Override
    public void periodic() {
        // double debugSpeed = SmartDashboard.getNumber("ShooterTestSpeed", 0);

        // if (debugSpeed != 0) {
        //     leftShooter.setShooterSpeed(debugSpeed);
        //     rightShooter.setShooterSpeed(debugSpeed);

        // } else {
        //     leftShooter.setShooterSpeed(Constants.ShooterConstants.IdleSpeed);
        //     rightShooter.setShooterSpeed(Constants.ShooterConstants.IdleSpeed);
        // }

        Pose2d baseTargetPose = getTargetPose();

        if (baseTargetPose != null) {
        // 1. Get current robot state
        Pose2d currentRobotPose = drivetrain.getState().Pose;

        // 2. Calculate LEFT Hood & Shooter
        calculateAndApplyAiming(currentRobotPose, leftturret, leftShooter, leftHood,
        "Left");

        // 3. Calculate RIGHT Hood & Shooter
        calculateAndApplyAiming(currentRobotPose, rightturret, rightShooter,
        rightHood, "Right");
        }
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
            calculatedHoodTicks = calculateHoodTicks(distanceMeters);
        }

        if (shooter != null) {
            shooter.setShooterSpeed(calculatedRPS);
        }

        if (hood != null) {
            hood.setTargetAngle(calculatedHoodTicks);
        }

        // Telemetry
        SmartDashboard.putNumber("AimingManager/" + sideName + "/Distance_m",
        distanceMeters);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/RPS",
        calculatedRPS);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/Hood Ticks",
        calculatedHoodTicks);
        SmartDashboard.putNumber("AimingManager/" + sideName + "/Hood Ticks Actual",
        hood.getEncoderTicks());
    }

    private double calculateHoodTicks(double distanceMeters) {
        
        // TODO Auto-generated method stub
        double a = 1.1917;
        double b = -3.8555;
        double c = 3.1307;
        return filter.calculate(a * distanceMeters * distanceMeters + b * distanceMeters + c);
    }

    private double calculateRps(double d) {
        // https://docs.google.com/spreadsheets/d/12vaU1FRqllZlERNKd85nal3VIQaEh6twuFeA2sOHeNw/edit?pli=1&gid=0#gid=0
        double a = 2.4464;
        double b = -2.0846;
        double c = 41.182;
        return a * d * d + b * d + c;
    }

    /**
     * Determines which Pose to aim at based on the Alliance color and
     * ZoneDetection.
     */
    private Pose2d getTargetPose() {
        if (zoneDetection == null)
            return null;

        var alliance = zoneDetection.getAlliance();
        if (alliance.isEmpty())
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
