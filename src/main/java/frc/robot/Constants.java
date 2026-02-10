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
        public static final int HopperCanId = 24;
    }
    public static final class IntakeConstants {}
    public static final class ShooterConstants {
        public static final int ShooterCanId1 = 21;
        public static final int ShooterCanId2 = 22;
        public static final int HoodCanId = 23;
    }
    public static final class TurretConstants {
        public static final int TurretCanId = 20;
        public static final double kP = 0.0012d;
        
        // Gear Ratio: 240/24 = 10:1 reduction
        public static final double TurretGearRatio = 24.0 / 240.0;
        
        public static final double MinAngle = -90.0;
        public static final double MaxAngle = 270.0;
    }

    public static final class FieldConstants {
        // Based on analysis: Original code had X/Y swapped relative to comments.
        // Comment: X=182.1" (4.62m), Y=159.1" (4.04m).
        // X=4.62m is on the BLUE side (Origin 0,0 is Blue Wall).
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
}

