package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor;
    private SwerveDrivetrain<?, ?, ?> DriveTrain;
    private ZoneDetection zoneDetection;
    private edu.wpi.first.math.geometry.Translation2d m_robotOffset;
    private CANcoder encoder;

    private PIDController turrentPID;

    public enum TURRENT_SIDE {
        RIGHT, LEFT
    };

    protected TURRENT_SIDE m_side;
    // y in inches: 159.1 = 4.0386 m
    // x in inches: 182.1 = 4.6228 m

    public Turret(int turretCanId, int encoderID, edu.wpi.first.math.geometry.Translation2d turretOffset,
            SwerveDrivetrain<?, ?, ?> drivetrain, ZoneDetection zoneDetection, TURRENT_SIDE side) {

        this.zoneDetection = zoneDetection;
        m_robotOffset = turretOffset;
        m_side = side;
        DriveTrain = drivetrain;

        encoder = new CANcoder(encoderID);
        TurretMotor = new TalonFX(turretCanId);

        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        // Tell the TalonFX to use the CANcoder for its position measurements
        turretConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        turretConfig.Feedback.FeedbackSensorSource = com.ctre.phoenix6.signals.FeedbackSensorSourceValue.RemoteCANcoder;

        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 2.0;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -2.0;

        TurretMotor.getConfigurator().apply(turretConfig);

        // We will no longer zero the encoder here because it is absolute.
        // encoder.setPosition(0);

        turrentPID = new PIDController(Constants.TurretConstants.kP, Constants.TurretConstants.kI,
                Constants.TurretConstants.kD);
        turrentPID.disableContinuousInput();
    }

    @Override
    public void periodic() {
        // --- 1. Sensors & State ---
        // Get current position in Rotations
        double currentMotorRotations = TurretMotor.getPosition().getValueAsDouble();
        // Convert to Degrees for Logic, factoring in the CANcoder's physical zero
        // offset
        double absoluteRotations = currentMotorRotations - TurretConstants.TurretAngleOffset;
        double currentTurretDegrees = rotationsToDegrees(absoluteRotations);

        double robotHeadingDegrees = (DriveTrain != null) ? DriveTrain.getState().Pose.getRotation().getDegrees() : 0.0;

        SmartDashboard.putNumber("Turret " + m_side.name() + "/Current Angle Degrees", currentTurretDegrees);
        SmartDashboard.putNumber("Turret " + m_side.name() + "/Robot Heading Degrees", robotHeadingDegrees);

        // --- 2. Determine Target Pose ---
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
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
            Pose2d turretFieldPose = currentRobotPose.transformBy(
                    new edu.wpi.first.math.geometry.Transform2d(m_robotOffset, new Rotation2d()));

            Translation2d delta = targetPose.getTranslation().minus(turretFieldPose.getTranslation());
            double targetFieldDegrees = delta.getAngle().getDegrees();

            // Calculate distance to target (norm of the translation difference)
            double distanceToTarget = delta.getNorm();
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Distance to Target (m)", distanceToTarget);

            // RobotHeading + TurretRelative = TargetField
            // TurretRelative = TargetField - RobotHeading
            double targetRelativeDegrees = targetFieldDegrees - robotHeadingDegrees;

            // Wrap the angle to handle the -180/180 degree boundary sign flip
            targetRelativeDegrees = MathUtil.inputModulus(targetRelativeDegrees, -180.0, 180.0);

            // Optimize the target angle to fit within the valid range of the Turret (-90 to
            // 90 for initial testing)
            double constrainedTargetDegrees = MathUtil.clamp(targetRelativeDegrees, TurretConstants.MinAngle,
                    TurretConstants.MaxAngle);

            SmartDashboard.putNumber("Turret " + m_side.name() + "/Target Relative Angle", targetRelativeDegrees);
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Constrained Angle", constrainedTargetDegrees);

            // Apply to Motor
            //setTargetAngle(constrainedTargetDegrees);
        } else {
            // Idle / Forward
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Distance to Target (m)", 0.0);
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Target Relative Angle", 0.0);
            SmartDashboard.putNumber("Turret " + m_side.name() + "/Constrained Angle", 0.0);
        }
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

        // convert to rotations
        double rotations = targetAngle / 45;

        double currentAbsRotations = encoder.getPosition().getValueAsDouble() - TurretConstants.TurretAngleOffset;
        double motoroutput = turrentPID.calculate(currentAbsRotations, rotations);

        motoroutput = MathUtil.clamp(motoroutput, -0.75, 75);

        TurretMotor.set(motoroutput);
    }

    /**
     * Returns a command that slowly turns the turret left manually.
     */
    public Command ManualTurnLeft() {
        return runOnce(() -> TurretMotor.set(0.1f));
    }

    /**
     * Returns a command that slowly turns the turret right manually.
     */
    public Command ManualTurnRight() {
        return runOnce(() -> TurretMotor.set(-0.1f));
    }

    /**
     * Returns a command that completely stops the turret motor.
     */
    public Command StopTurret() {
        return runOnce(() -> TurretMotor.set(0));
    }

    /**
     * Gets the current angle of the turret natively converted from the absolute
     * encoder.
     *
     * @return Current turret angle in degrees.
     */
    public double GetCurrentAngle() {
        double absoluteRotations = TurretMotor.getPosition().getValueAsDouble() - TurretConstants.TurretAngleOffset;
        return rotationsToDegrees(absoluteRotations);
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
}
