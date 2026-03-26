package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {

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
        public static final double L3 = 1.0; // ~3.3 feet
        public static final double Tolerance = 0.02; // 2cm tolerance
        public static final double SoftLimitBuffer = 0.05;
    }

    public static final class HopperConstants {
        public static final int FlopperCanID = 8;
        public static final int LeftTowerCANID = 9;
        public static final int RightTowerCANID = 10;

        public static final int CanRangeID1 = 4;
        public static final int CanRangeID2 = 5;
        // Interior space is ~10.25 inches (260mm).
        public static final double LaserMinDistance = 200.0;

        public static final double HopperFeedSpeed = 0.65;
        public static final double TowerFeedSpeed = 1.0;

        public static final int HopperCurrentLimit = 40;
        public static final double SplitterCurrentLimit = 30.0;
    }

    public static final class IntakeConstants {
        public static final int IntakeCanId = 7;
        // Speeds
        public static final double IntakeDutyCycle = 0.8; // 80% Power
        public static final double Min_Surface_Speed = 4.0; // 4.0 meters/second (approx 50-60% power)
        public static final double RobotSpeedMultiplier = 2d; // Surface speed = Robot Speed * 1.5
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
        public static final int IntakeDeployCanId = 6;
        public static final double kDeployP = 1.5;
        public static final double kDeployI = 0.0;
        public static final double kDeployD = 0.0;

        public static final double DeployCurrentLimit = 50.0;

        // Deploy Positions in Rotations (Placeholder)
        public static final double DeployPosition = 30.0; // Fully Extended
        public static final double RetractPosition = 0.0; // Fully Retracted (Home)
        public static final double DeployTolerance = 0.5;
        public static final double AgiPosition = 24.0;
    }

    public static final class HoodConstants {
        public static final int HoodCanId1 = 11;
        public static final int HoodCanId2 = 21;
        public static final double kP = 1.1;
        public static double kI = 0;
        public static double kD = 0;
    }

    public static final class ShooterConstants {
        public static final int ShooterCanId1 = 29;// left
        public static final int ShooterCanId2 = 28;

        public static final int ShooterCanId3 = 18;// right
        public static final int ShooterCanId4 = 19;

        public static double kP = 0.00003f;
        public static final double kV = 0.0025; // Volts per RPM. Roughly 12V / 5600 RPM for a NEO/Vortex

        // Target speeds in RPS (Rotations Per Second)
        public static final double IdleSpeed = 750.0 / 60.0; // 500 RPM
        public static final double ShootSpeed = 5500.0 / 60.0; // 3000 RPM
    }

    /** Constants for shoot-while-moving (velocity-based lead angle). */
    public static final class ShootingConstants {
        /** Estimated ball exit velocity in m/s for time-of-flight. Tune via testing. */
        public static final double BallSpeedMps = 12.0;
        /** Tunable scale for lead angle (1.0 = nominal). */
        public static final double LeadAngleScale = 1.0;
        /** Min robot velocity (m/s) to apply lead - below this use static aim. */
        public static final double VelocityThresholdMps = 0.3;
        /** Time-of-flight clamp (s) to avoid huge lead at point-blank. */
        public static final double MinTimeOfFlight = 0.1;
        public static final double MaxTimeOfFlight = 1.0;
        /** Base shooter RPM at close range. Scale up with distance. */
        public static final double BaseShooterRPM = 3600.0;
        /** RPM per meter of distance (linear scaling). */
        public static final double RpmPerMeter = 100.0;
        /** Base hood angle (deg) at close range. */
        public static final double BaseHoodAngleDeg = 15.0;
        /** Hood angle per meter of distance (deg). */
        public static final double HoodAnglePerMeter = 2.0;
    }

    public static final class TurretConstants {
        public static final int TurretCanId1 = 12; // right
        public static final int TurretCanId2 = 22; // left

        public static final edu.wpi.first.math.geometry.Translation2d TurretOffset1 = new edu.wpi.first.math.geometry.Translation2d(
                Units.inchesToMeters(-8.15), Units.inchesToMeters(-6.24)); // left
        public static final edu.wpi.first.math.geometry.Translation2d TurretOffset2 = new edu.wpi.first.math.geometry.Translation2d(
                Units.inchesToMeters(8.15), Units.inchesToMeters(-6.24)); // right

        public static final int encoderCanID1 = 3;
        public static final int encoderCanID2 = 2;

        public static final class LeftTurret {
            public static final double kP = 0.43d;
            public static final double kI = 0.0d;
            public static final double kD = 0.0d;
        }

        public static final class RightTurret {
            public static final double kP = 0.4d;
            public static final double kI = 0.0d;
            public static final double kD = 0.0d;
        }
        // public static final double kTolerance = 0.0; // Degrees

        public static final double CurrentLimit = 30.0;

        // Gear Ratio: 240/24 = 10:1 reduction
        public static final double TurretGearRatio = 1f / 8f;

        public static final double MinAngle = -90;
        public static final double MaxAngle = 90; // need to convert to angle and eventually use radians

        // Offset for the absolute CANcoder to treat its starting valid position as 0
        // degrees.
        // Measured in motor rotations. (45 degrees = 1 rotation).
        // public static final double TurrentRotationOffsetLeft = -0.00219;
        // public static final double TurrentRotationOffsetRight = -0.5055;
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
                4.62, 4.035, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(0));

        // Red Target (Mirrored / Rotated)
        // Standard Field Length approx 16.54m (54ft ish)
        // If Rotational Symmetry (180 deg rotation around center):
        // Red X = FieldLength - Blue X = 16.54 - 4.6228 = 11.9172
        // Red Y = FieldWidth - Blue Y = 8.21 - 4.0386 = 4.17
        public static final edu.wpi.first.math.geometry.Pose2d RedTargetPose = new edu.wpi.first.math.geometry.Pose2d(
                16.54105 - 4.62, 4.035, edu.wpi.first.math.geometry.Rotation2d.fromDegrees(180));

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