package frc.robot;



public final class Constants {
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
        public static final double p = 0.0012d;
    }
    public static final class BallTunnelConstants {
    
    }
}

