package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor;
    private SwerveDrivetrain DriveTrain;
    // private double ratio = 24.0f / 240.0f; // Moved to Constants
    public double target;
    // public double p; // Use Constants

    // y in inches: 159.1 = 4.0386 m
    // x in inches: 182.1 = 4.6228 m

    @SuppressWarnings("rawtypes")
    public Turret(SwerveDrivetrain drivetrain) {
        TurretMotor = new TalonFX(TurretConstants.TurretCanId);

        // Configure Soft Limits
        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        
        // Forward Limit: 270 degrees. Converted to Rotations:
        // 270 deg / 360 deg/rot = 0.75 turret rotations.
        // Motor Rotations = Turret Rotations / GearRatio
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (TurretConstants.MaxAngle / 360.0) / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // Reverse Limit: -90 degrees.
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (TurretConstants.MinAngle / 360.0) / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        TurretMotor.getConfigurator().apply(turretConfig);
        
        TurretMotor.setPosition(0);

        DriveTrain = drivetrain;
    }

    @Override
    public void periodic() {
        // Convert the Kraken encoder ticks to degrees
        // MotorRotations * GearRatio * 360 = Degrees
        double currentTurretDegrees = TurretMotor.getPosition().getValueAsDouble() * TurretConstants.TurretGearRatio * 360.0;
        
        // This is the robot's heading in field space (0 to 360 typically, or continuous)
        double robotHeadingDegrees = DriveTrain.getState().Pose.getRotation().getDegrees();

        // Calculate the turret's actual angle in field space
        // Example: Robot at 90, Turret at 0 (relative) -> Turret is pointing at 90 (field)
        double currentTurretFieldDegrees = robotHeadingDegrees + currentTurretDegrees;
        
        SmartDashboard.putNumber("Turret/Field Angle", currentTurretFieldDegrees);
        SmartDashboard.putNumber("Turret/Relative Angle", currentTurretDegrees);

        // Calculate the angle we WANT to be at in field space
        // double targetFieldDegrees = CalculateAngleToTarget(); // Moved to conditional below
        double targetRelativeDegrees = 0.0;
        
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        boolean shouldTrack = false;
        
        if (alliance.isPresent()) {
            var color = alliance.get();
            var zone = zoneDetection.getZone(); // myZone is public, or check getter
            
            // Check if we are in our Home Zone
            // Blue Alliance matches BLUE Zone
            // Red Alliance matches RED Zone
            if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue && zone == ZoneDetection.ZONE.BLUE) {
                shouldTrack = true;
            } else if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Red && zone == ZoneDetection.ZONE.RED) {
                shouldTrack = true;
            }
        }
        
        if (shouldTrack) {
             // Calculate the field space delta in turret angle
             double targetFieldDegrees = CalculateAngleToTarget();
             SmartDashboard.putNumber("Turret/Target Field Angle", targetFieldDegrees);
             
             // TargetRelative = TargetField - RobotHeading
             targetRelativeDegrees = targetFieldDegrees - robotHeadingDegrees;
        } else {
            // Aim Forward (0 degrees relative to robot)
            targetRelativeDegrees = 0.0;
            SmartDashboard.putNumber("Turret/Target Field Angle", robotHeadingDegrees); // Effectively aiming at robot heading
        }

        SmartDashboard.putBoolean("Turret/Tracking", shouldTrack);

        // Normalize the target relative angle to be within valid range logic if needed, 
        // but for a limited turret, we usually want to find the nearest solution that is within limits.
        // However, since we have hard stops, we can't just wrap 180 to -180 arbitrarily if it crosses the "dead zone".
        
        // For now, let's keep the error calculation simple but robust:
        double error = targetRelativeDegrees - currentTurretDegrees;
        
        // Normalize error to shortest path (-180 to 180)
        // This allows the turret to cross 0 efficiently
        error = MathUtil.inputModulus(error, -180.0, 180.0);

        SmartDashboard.putNumber("Turret/Error", error);

        double power = error * TurretConstants.kP;          
        TurretMotor.set(power);
    }

    public void setPower(float speed) {
        TurretMotor.set(speed);
    }

    public Command ManualTurnLeft() {
        return runOnce(() -> TurretMotor.set(0.1f));
    }

    public Command ManualTurnRight() {
        return runOnce(() -> TurretMotor.set(-0.1f));
    }

    public Command StopTurret() {
        return runOnce(() -> TurretMotor.set(0));
    }

    public double GetCurrentAngle() {
        return TurretMotor.getPosition().getValueAsDouble() * TurretConstants.TurretGearRatio * 360.0f;
    }

    public double CalculateAngleToTarget() {
        Pose2d robotPose = DriveTrain.getState().Pose;
        
        // Default to "Red" logic (or whatever was hardcoded) if no alliance, 
        // but ideally we check DriverStation.
        Pose2d targetPose = Constants.FieldConstants.BlueTargetPose; // Default to Blue (original coords seemed to be Blue side)

        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent()) {
            if (alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                targetPose = Constants.FieldConstants.RedTargetPose;
            } else {
                targetPose = Constants.FieldConstants.BlueTargetPose;
            }
        }

        Translation2d deltaVector = targetPose.getTranslation().minus(robotPose.getTranslation());
        Rotation2d angleToTarget = deltaVector.getAngle();

        return angleToTarget.getDegrees();
    }
}
