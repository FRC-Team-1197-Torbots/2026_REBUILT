package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    public static final class LimeLightConstants {
        public static final String limelightname = "limelight-alpha";
    }

    public static final class DriveConstants {
    }

    public static final class ModuleConstants {
    }

    public static final class AutoConstants {
    }

    public static final class ClimberConstants {
        public static final Pose2d lowerClimbPosition = new Pose2d(1.066, 2.782, Rotation2d.fromDegrees(90));
        
        public static final int ClimberCanId = 30;
        public static final int ClimberDateId = 31; // CANrange

        public static final double kP = 0.1; 
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        
        // Mechanical Constants
        public static final double ClimberGearRatio = 12.0; // Example: 12:1
        public static final double ClimberSpoolDiameter = 0.0508; // Example: 2 inches in meters
        
        // Climber Heights in Meters
        public static final double L0 = 0.0;
        public static final double L1 = 0.30; // ~1 foot
        public static final double L2 = 0.60; // ~2 feet
        public static final double L3 = 1.0;  // ~3.3 feet
        public static final double Tolerance = 0.02; // 2cm tolerance
        public static final double SoftLimitBuffer = 0.05;
    }

    public static final class HopperConstants {
        public static final int HopperCanId1 = 5;
        public static final int HopperCanId2 = 6;
        public static final int HooperSplitterCanID = 7;
        
        public static final int HooperLaserCANID = 8;
        // Interior space is ~10.25 inches (260mm).
        public static final double LaserMinDistance = 200.0;

        public static final double HopperFeedSpeed = 0.6;
        
        public static final int HopperCurrentLimit = 40;
        public static final double SplitterCurrentLimit = 30.0;
    }

    public static final class IntakeConstants {
        public static final int IntakeCanId = 25;
        // Speeds
        public static final double IntakeDutyCycle = 0.8; // 80% Power
        public static final double Min_Surface_Speed = 4.0; // 4.0 meters/second (approx 50-60% power)
        public static final double RobotSpeedMultiplier = 1.5d; // Surface speed = Robot Speed * 1.5
        public static final double OuttakeSpeed = -0.4;

        public static final double wheelDiameter = 0.75;
        public static final double gearratio = 1; // Direct drive

        // Kraken X44 Constants
        // Free speed ~7400 RPM => ~123 RPS. kV = 12V / 123RPS ≈ 0.097
        public static final double kP = 0.08; // Starting value for velocity control
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.10; // ~0.1V per RPS
        
        public static final double RollerCurrentLimit = 40.0;

        // Intake Deploy (Rack & Pinion)
        public static final int IntakeDeployCanId = 26;
        public static final double kDeployP = 0.1;
        public static final double kDeployI = 0.0;
        public static final double kDeployD = 0.0;
        
        public static final double DeployCurrentLimit = 30.0;
        
        // Deploy Positions in Rotations (Placeholder)
        public static final double DeployPosition = 10.0; // Fully Extended
        public static final double RetractPosition = 0.0;  // Fully Retracted (Home)
        public static final double DeployTolerance = 0.5;
    }
    
    public static final class HoodConstants {
        public static final int HoodCanId = 23;
        public static final double kP = 0.1;
        public static final double DegreesPerRotation = (1.0 / 18.0) * 360.0;
    }

    public static final class ShooterConstants {
        public static final int ShooterCanId1 = 21;
        public static final int ShooterCanId2 = 22;

        public static final double kP = 0.11;
        public static final double kV = 0.12;
        public static final double CurrentLimit = 80.0;
        
        public static final double IdleSpeed = 15.0; // RPS (~900 RPM)
    }

    public static final class TurretConstants {
        public static final int TurretCanId = 20;
        public static final double kP = 0.0012d;
        public static final double kI = 0.0d;
        public static final double kD = 0.0d;
        public static final double kTolerance = 1.0; // Degrees
        
        public static final double CurrentLimit = 30.0;

        // Gear Ratio: 240/24 = 10:1 reduction
        public static final double TurretGearRatio = 24.0 / 240.0;

        public static final double MinAngle = -90.0;
        public static final double MaxAngle = 270.0;        

    }

    public static final class FieldConstants {
        public static final double FieldLength = 16.541;
        public static final double FieldWidth = 8.211;

        // Passing Targets (Aim Points near Feeder Station / Corners)
        // Offset by ~1.5m to ensure we don't shoot off the field
        public static final double PassingMargin = 1.5;

        // 2026 Manual: 156.61 inches
        public static final double BlueAllianceLineX = edu.wpi.first.math.util.Units.inchesToMeters(156.61);
        public static final double RedAllianceLineX = FieldLength - BlueAllianceLineX;
        // Based on analysis: Original code had X/Y swapped relative to comments.
        // Comment: X=182.1" (4.62m), Y=159.1" (4.04m).
        // Y=4.04m is roughly CENTER field width (8.2m total width).

        // Blue Target (The one on the Blue Side)
        // Note: In some games you shoot at your OWN side (Tower?), in others OPPOSITE
        // (Speaker).
        // Assuming X=4.62 is the specific target location:
        public static final edu.wpi.first.math.geometry.Pose2d BlueTargetPose = new edu.wpi.first.math.geometry.Pose2d(
                4.6228, 4.0386, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));

        // Red Target (Mirrored / Rotated)
        // Standard Field Length approx 16.54m (54ft ish)
        // If Rotational Symmetry (180 deg rotation around center):
        // Red X = FieldLength - Blue X = 16.54 - 4.6228 = 11.9172
        // Red Y = FieldWidth - Blue Y = 8.21 - 4.0386 = 4.17
        public static final edu.wpi.first.math.geometry.Pose2d RedTargetPose = new edu.wpi.first.math.geometry.Pose2d(
                16.541 - 4.6228, 8.211 - 4.0386, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));

        // Blue Passing (Aiming at Blue Wall X=0)
        public static final edu.wpi.first.math.geometry.Pose2d BluePassingCornerRight = new edu.wpi.first.math.geometry.Pose2d(
                PassingMargin, PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
        public static final edu.wpi.first.math.geometry.Pose2d BluePassingCornerLeft = new edu.wpi.first.math.geometry.Pose2d(
                PassingMargin, FieldWidth - PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));

        // Red Passing (Aiming at Red Wall X=Length)
        public static final edu.wpi.first.math.geometry.Pose2d RedPassingCornerRight = new edu.wpi.first.math.geometry.Pose2d(
                FieldLength - PassingMargin, PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
        public static final edu.wpi.first.math.geometry.Pose2d RedPassingCornerLeft = new edu.wpi.first.math.geometry.Pose2d(
                FieldLength - PassingMargin, FieldWidth - PassingMargin,
                edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
    }

}
