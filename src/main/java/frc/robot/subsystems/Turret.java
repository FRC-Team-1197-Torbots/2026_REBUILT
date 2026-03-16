package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor;
    private SwerveDrivetrain<?, ?, ?> DriveTrain;
    private ZoneDetection zoneDetection;
    private edu.wpi.first.math.geometry.Translation2d m_robotOffset;
    private edu.wpi.first.math.geometry.Transform2d m_turretOffsetTransform;

    private PIDController turrentPID;
    private double TargetRotations;
    
    // Cache for Alliance to reduce JNI overhead
    private java.util.Optional<edu.wpi.first.wpilibj.DriverStation.Alliance> m_alliance = java.util.Optional.empty();
    private int m_allianceCheckDelay = 0;

    public enum TURRET_SIDE {
        RIGHT, LEFT
    };

    protected TURRET_SIDE m_side;

    public Turret(int turretCanId, edu.wpi.first.math.geometry.Translation2d turretOffset,
            SwerveDrivetrain<?, ?, ?> drivetrain, ZoneDetection zoneDetection, TURRET_SIDE side) {

        this.zoneDetection = zoneDetection;
        m_robotOffset = turretOffset;
        m_side = side;
        DriveTrain = drivetrain;

        // Pre-compute the transform here so we don't allocate it in periodic() every 20ms
        // A Transform2d without a Rotational component simply translates the origin point
        m_turretOffsetTransform = new edu.wpi.first.math.geometry.Transform2d(m_robotOffset, new Rotation2d());

        TurretMotor = new TalonFX(turretCanId);

        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        // Set Neutral Mode to Brake
        turretConfig.MotorOutput.NeutralMode = com.ctre.phoenix6.signals.NeutralModeValue.Brake;

        TurretMotor.getConfigurator().apply(turretConfig);

        TargetRotations = getRelativeRotation();

        if(side == TURRET_SIDE.LEFT) {
            turrentPID = new PIDController(Constants.TurretConstants.LeftTurret.kP, Constants.TurretConstants.LeftTurret.kI,
                Constants.TurretConstants.LeftTurret.kD);
        } else {
            turrentPID = new PIDController(Constants.TurretConstants.RightTurret.kP, Constants.TurretConstants.RightTurret.kI,
                Constants.TurretConstants.RightTurret.kD);
        }

        turrentPID.disableContinuousInput();
    }

    @Override
    public void periodic() {

        // --- 1. Sensors & State ---
        // Get current position in Rotations
        double currentMotorRotations = TurretMotor.getPosition().getValueAsDouble();
        
        // Convert to Degrees for Logic, factoring in the CANcoder's physical zero offset
        double absoluteRotations = getRelativeRotation();
        double currentTurretDegrees = rotationsToDegrees(absoluteRotations);

        double robotHeadingDegrees = (DriveTrain != null) ? DriveTrain.getState().Pose.getRotation().getDegrees() : 0.0;

        // --- 2. Determine Target Pose ---
        // Cache alliance to avoid expensive JNI call every 20ms
        if (m_allianceCheckDelay <= 0) {
            m_alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
            m_allianceCheckDelay = 50; // Check once per second
        } else {
            m_allianceCheckDelay--;
        }
        var alliance = m_alliance;
        
        Pose2d targetPose = null;
        boolean shouldTrack = false;

        if (alliance.isPresent() && zoneDetection != null && DriveTrain != null &&
                m_robotOffset != null) {
            var color = alliance.get();
            var zone = zoneDetection.getZone();

            if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) {
                if (zone == ZoneDetection.ZONE.BLUE) {
                    // Home Zone -> Attack Hub
                    targetPose = Constants.FieldConstants.BlueTargetPose;
                    shouldTrack = true;
                } else if (zone == ZoneDetection.ZONE.NEUTRAL) {
                    // Neutral Zone -> Pass to Corner (Safe)
                    // Logic: If on Right side(Y < Width/2) -> Right Corner. Else Left Corner.
                    if (DriveTrain.getState().Pose.getY() < Constants.FieldConstants.FieldWidth /
                            2.0) {
                        targetPose = Constants.FieldConstants.BluePassingCornerRight;
                    } else {
                        targetPose = Constants.FieldConstants.BluePassingCornerLeft;
                    }
                    shouldTrack = true;
                }
            } else if (color == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
                if (zone == ZoneDetection.ZONE.RED) {
                    // Home Zone -> Attack Hub
                    targetPose = Constants.FieldConstants.RedTargetPose;
                    shouldTrack = true;
                } else if (zone == ZoneDetection.ZONE.NEUTRAL) {
                    // Neutral Zone -> Pass to Corner (Safe)
                    if (DriveTrain.getState().Pose.getY() < Constants.FieldConstants.FieldWidth /
                            2.0) {
                        targetPose = Constants.FieldConstants.RedPassingCornerRight;
                    } else {
                        targetPose = Constants.FieldConstants.RedPassingCornerLeft;
                    }
                    shouldTrack = true;
                }
            }
        }

        // --- 3. Calculate Desired Angle & Apply Control ---
        if (shouldTrack && targetPose != null) {
            Pose2d currentRobotPose = DriveTrain.getState().Pose;
            Pose2d turretFieldPose = currentRobotPose.transformBy(m_turretOffsetTransform);            

            Translation2d delta = targetPose.getTranslation().minus(turretFieldPose.getTranslation());
            double targetFieldDegrees = delta.getAngle().getDegrees();

            // Calculate distance to target (norm of the translation difference)
            double distanceToTarget = delta.getNorm();
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Distance to Target (m)", distanceToTarget);

            // RobotHeading + TurretRelative = TargetField
            // TurretRelative = TargetField - RobotHeading + 180 (Since the turrets are backwards)
            double targetRelativeDegrees = targetFieldDegrees - robotHeadingDegrees + 180.0;

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

        SmartDashboard.putNumber("Turrent" + m_side.name() +"/Target Rotation", TargetRotations);
        SmartDashboard.putNumber("Turrent" + m_side.name() +"/Actual Rotation", TurretMotor.getPosition().getValueAsDouble());
    }

    public void EvaluateTurret() {              
        double currentAbsRotations = getRelativeRotation();
        double motoroutput = turrentPID.calculate(currentAbsRotations, TargetRotations);

        // Enforce soft limits
        double minLimit = (m_side == TURRET_SIDE.LEFT) ? TurretConstants.LeftTurret.MinRotationLimit : TurretConstants.RightTurret.MinRotationLimit;
        double maxLimit = (m_side == TURRET_SIDE.LEFT) ? TurretConstants.LeftTurret.MaxRotationLimit : TurretConstants.RightTurret.MaxRotationLimit;

        if (currentAbsRotations <= minLimit && motoroutput < 0) {
            motoroutput = 0; // Prevent driving further negative
        } else if (currentAbsRotations >= maxLimit && motoroutput > 0) {
            motoroutput = 0; // Prevent driving further positive
        }

        // Commented out to allow manual movement for finding limits
        // TurretMotor.set(motoroutput);
    }

    /**
     * Attempts to turn the turret to a specific target angle using closed-loop PID.
     * Limits the command to valid angle bounds.
     * 
     * @param targetAngle The target angle in degrees relative to the robot's front
     */
    public void setTargetAngle(double targetAngle) {
        if (Math.abs(targetAngle) > 90)
            return;

        TargetRotations = degreesToRotations(targetAngle);
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
        return TurretMotor.getPosition().getValueAsDouble();
    }
}
