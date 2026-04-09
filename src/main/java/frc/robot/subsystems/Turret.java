package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Intake.INTAKE_POSITION;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    private double TurrentRotationOffset = 0;

    private TalonFX TurretMotor;
    private SwerveDrivetrain<?, ?, ?> DriveTrain;
    private ZoneDetection zoneDetection;
    private edu.wpi.first.math.geometry.Translation2d m_robotOffset;
    private edu.wpi.first.math.geometry.Transform2d m_turretOffsetTransform;
    private CANcoder encoder;

    private PIDController turrentPID;
    private double TargetRotations;

    // Globally cached distance and tracking state
    private double m_distanceToTarget = 0.0;
    private final String m_distanceLogKey;

    public enum TURRET_SIDE {
        RIGHT, LEFT
    };

    protected TURRET_SIDE m_side;
    private Intake m_Intake;

    public Pose2d targetPose;
    // Legacy piece speed, replaced by calculateTimeOfFlight regression curve
    // private final double AVERAGE_PIECE_SPEED_MPS = 10.0; // Needs tuning
    public boolean enableShootOnTheMove = true;

    public Turret(int turretCanId, int encoderID, edu.wpi.first.math.geometry.Translation2d turretOffset,
            SwerveDrivetrain<?, ?, ?> drivetrain, ZoneDetection zoneDetection, TURRET_SIDE side, Intake intake) {

        this.zoneDetection = zoneDetection;
        m_robotOffset = turretOffset;
        m_side = side;
        DriveTrain = drivetrain;

        // Pre-compute the transform here so we don't allocate it in periodic() every
        // 20ms
        // A Transform2d without a Rotational component simply translates the origin
        // point
        m_turretOffsetTransform = new edu.wpi.first.math.geometry.Transform2d(m_robotOffset, new Rotation2d());

        encoder = new CANcoder(encoderID);
        encoder.setPosition(0);
        TargetRotations = 0;

        TurrentRotationOffset = encoder.getPosition().getValueAsDouble();

        TurretMotor = new TalonFX(turretCanId);

        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        // Set Neutral Mode to Brake
        turretConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        TurretMotor.getConfigurator().apply(turretConfig);

        if (side == TURRET_SIDE.LEFT) {
            turrentPID = new PIDController(Constants.TurretConstants.LeftTurret.kP,
                    Constants.TurretConstants.LeftTurret.kI,
                    Constants.TurretConstants.LeftTurret.kD);
        } else {
            turrentPID = new PIDController(Constants.TurretConstants.RightTurret.kP,
                    Constants.TurretConstants.RightTurret.kI,
                    Constants.TurretConstants.RightTurret.kD);
        }

        turrentPID.disableContinuousInput();

        m_Intake = intake;

        m_distanceLogKey = "Turret" + m_side.name() + "/Distance to Target";
        // setTargetAngle(0);
    }

    @Override
    public void periodic() {
        // --- 1. Sensors & State ---
        // Get current position in Rotations
        double currentMotorRotations = TurretMotor.getPosition().getValueAsDouble();

        // Convert to Degrees for Logic, factoring in the CANcoder's physical zero
        // offset
        double absoluteRotations = getRelativeRotation();
        double currentTurretDegrees = rotationsToDegrees(absoluteRotations);

        double robotHeadingDegrees = (DriveTrain != null) ? DriveTrain.getState().Pose.getRotation().getDegrees() : 0.0;

        // --- 2. Determine Target Pose ---
        // Grab globally cached alliance
        var alliance = (zoneDetection != null) ? zoneDetection.getAlliance()
                : java.util.Optional.<edu.wpi.first.wpilibj.DriverStation.Alliance>empty();

        boolean shouldTrack = false;

        if (alliance.isPresent() && zoneDetection != null && DriveTrain != null && m_robotOffset != null) {
            var color = alliance.get();
            var zone = zoneDetection.getZone();

            boolean isBlue = color == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
            boolean isRed = color == edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

            if (isBlue && zone == ZoneDetection.ZONE.BLUE) {
                // Home Zone -> Attack Hub
                targetPose = Constants.FieldConstants.BlueTargetPose;
                shouldTrack = true;

            } else if (isRed && zone == ZoneDetection.ZONE.RED) {
                // Home Zone -> Attack Hub
                targetPose = Constants.FieldConstants.RedTargetPose;
                shouldTrack = true;

            } else if (zone == ZoneDetection.ZONE.NEUTRAL && (isBlue || isRed) && DriverStation.isTeleop()) {
                // Neutral Zone -> Pass to Corner (Safe)
                Pose2d passRight = isBlue ? Constants.FieldConstants.BluePassingCornerLeft
                        : Constants.FieldConstants.RedPassingCornerRight;
                Pose2d passLeft = isBlue ? Constants.FieldConstants.BluePassingCornerRight
                        : Constants.FieldConstants.RedPassingCornerLeft;

                // The 2026 REBUILT Hub funnel has a width of ~41 inches (1.0414 meters)
                double hubWidthMeters = Units.inchesToMeters(47);
                double centerY = Constants.FieldConstants.FieldWidth / 2.0;
                double middleFieldYMin = centerY - (hubWidthMeters / 2.0);
                double middleFieldYMax = centerY + (hubWidthMeters / 2.0);

                double robotY = DriveTrain.getState().Pose.getY();

                if (robotY >= middleFieldYMin && robotY <= middleFieldYMax) {
                    // Split shooting behind hubs
                    // NOTE: Corners are "flipped" (LEFT targets Right, RIGHT targets Left)
                    // because the turrets are mounted on the backside of the robot.
                    targetPose = (m_side == TURRET_SIDE.LEFT) ? passRight : passLeft;
                } else {
                    // Default Side Passing
                    // If on the Y=0 side of the field, target the Y=0 corner (Right Corner for
                    // Blue).
                    if (isRed)
                        targetPose = (robotY < centerY) ? passRight : passLeft;
                    if (isBlue)
                        targetPose = (robotY < centerY) ? passLeft : passRight;
                }

                shouldTrack = true;

            } else {
                // Opponent Zone -> Zero turrets
                shouldTrack = false;
            }
        }

        if (m_Intake.m_position == INTAKE_POSITION.RETRACTED || m_Intake.m_position == INTAKE_POSITION.RETRACTING) {
            shouldTrack = false;
        }
        // SmartDashboard.putBoolean("Should track", shouldTrack);
        // SmartDashboard.putString("Intake State", m_Intake.m_position.toString());
        // SmartDashboard.putNumber(m_distanceLogKey, m_distanceToTarget);
        // --- 3. Calculate Desired Angle & Apply Control ---
        if (shouldTrack && targetPose != null) {
            Pose2d currentRobotPose = DriveTrain.getState().Pose;

            // Primitive transform math saves 4 object allocations per 20ms over
            // .transformBy() and .minus()
            double cos = currentRobotPose.getRotation().getCos();
            double sin = currentRobotPose.getRotation().getSin();
            double turretX = currentRobotPose.getX() + (m_robotOffset.getX() * cos - m_robotOffset.getY() * sin);
            double turretY = currentRobotPose.getY() + (m_robotOffset.getX() * sin + m_robotOffset.getY() * cos);

            // To disable Shoot-On-The-Move structurally, simply comment out or delete the
            // assignment below.
            // The code will gracefully fall back to the stationary targetPose without any
            // compilation errors.
            Pose2d adjustedTarget = targetPose;
            adjustedTarget = applyShootOnTheMove(currentRobotPose, targetPose);

            double dX = adjustedTarget.getX() - turretX;
            double dY = adjustedTarget.getY() - turretY;

            double trueDx = targetPose.getX() - turretX;
            double trueDy = targetPose.getY() - turretY;

            // distanceToTarget is cached physically
            m_distanceToTarget = Math.hypot(dX, dY);
            double targetFieldDegrees = Math.toDegrees(Math.atan2(dY, dX));
            double trueTargetFieldDegrees = Math.toDegrees(Math.atan2(trueDy, trueDx));

            // Calculate the raw difference between where the target is and where the robot
            // is facing
            double headingDifference = MathUtil.inputModulus(trueTargetFieldDegrees - robotHeadingDegrees, -180.0,
                    180.0);

            double targetRelativeDegrees;

            // If the robot is generally facing the hub (+/- 90 deg), reset turrets to 0.
            // This is geometrically fixed at 90 degrees since the turrets are mounted
            // backward,
            // independent of the physical cable limits.
            if (Math.abs(headingDifference) < 90.0) {
                targetRelativeDegrees = 0.0;
            } else {
                // RobotHeading + TurretRelative = TargetField
                // TurretRelative = TargetField - RobotHeading + 180 (Since the turrets are
                // backwards)
                targetRelativeDegrees = targetFieldDegrees - robotHeadingDegrees + 180.0;
            }

            // Wrap the angle to handle the -180/180 degree boundary sign flip
            targetRelativeDegrees = MathUtil.inputModulus(targetRelativeDegrees, -180.0, 180.0);

            // Optimize the target angle to fit within the valid range of the Turret
            // (-90 to 90 degrees based on Constants)
            double constrainedTargetDegrees = MathUtil.clamp(targetRelativeDegrees, TurretConstants.MinAngle,
                    TurretConstants.MaxAngle);

            // Apply to Motor
            setTargetAngle(constrainedTargetDegrees);
        } else {
            setTargetAngle(0);
        }

        EvaluateTurret();
    }

    public void EvaluateTurret() {
        // double currentAbsRotations = getRelativeRotation();
        double motoroutput = turrentPID.calculate(encoder.getPosition().getValueAsDouble(), TargetRotations);

        // SmartDashboard.putNumber("Turrent" + m_side.name() + "/Target Rotation", TargetRotations);
        // SmartDashboard.putNumber("Turrent" + m_side.name() + "/Actual Rotation",
        //         encoder.getPosition().getValueAsDouble());

        TurretMotor.set(motoroutput);
    }

    /**
     * Attempts to turn the turret to a specific target angle using closed-loop PID.
     * Limits the command to valid angle bounds.
     * 
     * @param targetAngle The target angle in degrees relative to the robot's front
     */
    public void setTargetAngle(double targetAngle) {
        double clampedAngle = edu.wpi.first.math.MathUtil.clamp(targetAngle, TurretConstants.MinAngle,
                TurretConstants.MaxAngle);
        TargetRotations = degreesToRotations(clampedAngle);
    }

    /**
     * Helper function to convert natively from encoder rotations to degrees.
     * 1 rotation = 45 degrees.
     *
     * @param rotations The rotation measurement provided by the motor or the
     *                  encoder
     * @return The equivalent degrees
     */
    public double rotationsToDegrees(double rotations) {
        return rotations * 45.0;
    }

    public double degreesToRotations(double degrees) {
        return degrees / 45;
    }

    public double getRelativeRotation() {
        return encoder.getPosition().getValueAsDouble() - TurrentRotationOffset;
    }

    public boolean isStraight() {
        double currentDegrees = rotationsToDegrees(getRelativeRotation());
        return Math.abs(currentDegrees) < 10.0; // Within 10 degrees of straight
    }

    public double getDistanceToTarget() {
        return m_distanceToTarget;
    }

    public void zeroTurret() {
        encoder.setPosition(0);
    }

    private Pose2d applyShootOnTheMove(Pose2d robotPose, Pose2d targetPose) {
        if (!enableShootOnTheMove || targetPose == null) {
            return targetPose;
        }

        edu.wpi.first.math.kinematics.ChassisSpeeds speeds = DriveTrain.getState().Speeds;

        // Convert robot-relative speeds to field-relative speeds
        edu.wpi.first.math.geometry.Rotation2d robotHeading = robotPose.getRotation();
        double fieldVx = speeds.vxMetersPerSecond * robotHeading.getCos() - speeds.vyMetersPerSecond * robotHeading.getSin();
        double fieldVy = speeds.vxMetersPerSecond * robotHeading.getSin() + speeds.vyMetersPerSecond * robotHeading.getCos();

        double distance = targetPose.getTranslation().getDistance(robotPose.getTranslation());
        double timeOfFlight = calculateTimeOfFlight(distance);

        double offsetX = fieldVx * timeOfFlight;
        double offsetY = fieldVy * timeOfFlight;

        return new Pose2d(
                targetPose.getX() - offsetX,
                targetPose.getY() - offsetY,
                targetPose.getRotation());
    }

    /**
     * Calculates the estimated Time of Flight of the game piece based on target
     * distance.
     * Tune a, b, c by measuring actual time of flight at different distances.
     * 
     * @param distance The distance to the target in meters
     * @return Estimated Time of Flight in seconds
     */
    private double calculateTimeOfFlight(double distance) {
        // Defaults to a flat ~10 m/s horizontal speed if left untuned.
        // Replace a, b, c with your regression values when you have time to test.
        double a = 0.0;
        double b = 0.1; 
        double c = 0.0;

        return (a * distance * distance) + (b * distance) + c;
    }
}