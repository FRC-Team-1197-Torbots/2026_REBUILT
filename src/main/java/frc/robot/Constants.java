package frc.robot;

public final class Constants {
    public static final class LimeLightConstants{
        public static final String limelightname = "limelight-alpha"; 
    }
    public static final class DriveConstants {}
    public static final class ModuleConstants {}
    public static final class AutoConstants {}
    public static final class ClimberConstants {}
    public static final class HopperConstants {
        public static final int HopperCanId1 = 5;
        public static final int HopperCanId2 = 6;
        public static final int HooperSplitterCanID = 7;
        public static final double HopperFeedSpeed = 0.6;
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
    }
    public static final class ShooterConstants {
        public static final int ShooterCanId1 = 21;
        public static final int ShooterCanId2 = 22;
        public static final int HoodCanId = 23;
    }
    public static final class TurretConstants {
        public static final int TurretCanId = 20;
        public static final double kP = 0.0012d;
        public static final double kI = 0.0d;
        public static final double kD = 0.0d;
        public static final double kTolerance = 1.0; // Degrees
        
        // Gear Ratio: 240/24 = 10:1 reduction
        public static final double TurretGearRatio = 24.0 / 240.0;
        
        public static final double MinAngle = -90.0;
        public static final double MaxAngle = 270.0;

        public static final double FieldLength = 16.541;
        public static final double FieldWidth = 8.211;
        
        // 2026 Manual: 156.61 inches
        public static final double BlueAllianceLineX = edu.wpi.first.math.util.Units.inchesToMeters(156.61); 
        public static final double RedAllianceLineX = FieldLength - BlueAllianceLineX;
        
        // Passing Targets (Aim Points near Feeder Station / Corners)
        // Offset by ~1.5m to ensure we don't shoot off the field
        public static final double PassingMargin = 1.5;
        
        // Blue Passing (Aiming at Blue Wall X=0)
        public static final edu.wpi.first.math.geometry.Pose2d BluePassingCornerRight = 
            new edu.wpi.first.math.geometry.Pose2d(PassingMargin, PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
        public static final edu.wpi.first.math.geometry.Pose2d BluePassingCornerLeft = 
            new edu.wpi.first.math.geometry.Pose2d(PassingMargin, FieldWidth - PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));
            
        // Red Passing (Aiming at Red Wall X=Length)
        public static final edu.wpi.first.math.geometry.Pose2d RedPassingCornerRight = 
            new edu.wpi.first.math.geometry.Pose2d(FieldLength - PassingMargin, PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
        public static final edu.wpi.first.math.geometry.Pose2d RedPassingCornerLeft = 
            new edu.wpi.first.math.geometry.Pose2d(FieldLength - PassingMargin, FieldWidth - PassingMargin, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
    }

    public static final class FieldConstants {
        // Based on analysis: Original code had X/Y swapped relative to comments.
        // Comment: X=182.1" (4.62m), Y=159.1" (4.04m).
        // Y=4.04m is roughly CENTER field width (8.2m total width).
        
        // Blue Target (The one on the Blue Side)
        // Note: In some games you shoot at your OWN side (Tower?), in others OPPOSITE (Speaker).
        // Assuming X=4.62 is the specific target location:
        public static final edu.wpi.first.math.geometry.Pose2d BlueTargetPose = 
            new edu.wpi.first.math.geometry.Pose2d(4.6228, 4.0386, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));

        // Red Target (Mirrored / Rotated)
        // Standard Field Length approx 16.54m (54ft ish)
        // If Rotational Symmetry (180 deg rotation around center):
        // Red X = FieldLength - Blue X = 16.54 - 4.6228 = 11.9172
        // Red Y = FieldWidth - Blue Y = 8.21 - 4.0386 = 4.17
        public static final edu.wpi.first.math.geometry.Pose2d RedTargetPose = 
            new edu.wpi.first.math.geometry.Pose2d(16.541 - 4.6228, 8.211 - 4.0386, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));
    }
    public static final class BallTunnelConstants {
    
    }
}

