package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.ShootingKinematics;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor;
    private SwerveDrivetrain<?, ?, ?> DriveTrain;
    private ZoneDetection zoneDetection;
    private edu.wpi.first.math.geometry.Translation2d m_robotOffset;

    public enum TURRENT_SIDE {
        RIGHT, LEFT
    };

    protected TURRENT_SIDE m_side;

    // TalonFX Control Request
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // y in inches: 159.1 = 4.0386 m
    // x in inches: 182.1 = 4.6228 m

    /** Simple constructor for testing without full dependencies */
    public Turret(int turretCanId) {
        TurretMotor = new TalonFX(turretCanId);

        // Configure TalonFX
        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        turretConfig.Slot0.kP = TurretConstants.kP;
        turretConfig.Slot0.kI = TurretConstants.kI;
        turretConfig.Slot0.kD = TurretConstants.kD;

        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (TurretConstants.MaxAngle / 360.0)
                / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (TurretConstants.MinAngle / 360.0)
                / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.CurrentLimit;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TurretMotor.getConfigurator().apply(turretConfig);
        TurretMotor.setPosition(0);
    }

    public Turret(int turretCanId, edu.wpi.first.math.geometry.Translation2d robotOffset,
            SwerveDrivetrain<?, ?, ?> drivetrain, ZoneDetection zoneDetection, TURRENT_SIDE side) {
        this.zoneDetection = zoneDetection;
        m_robotOffset = robotOffset;
        TurretMotor = new TalonFX(turretCanId);
        m_side = side;

        // Configure TalonFX
        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        // Slot 0 PID config
        turretConfig.Slot0.kP = TurretConstants.kP;
        turretConfig.Slot0.kI = TurretConstants.kI;
        turretConfig.Slot0.kD = TurretConstants.kD;
        // Note: kTolerance is handled by the closed loop logic manually or by just
        // checking error,
        // TalonFX doesn't have a "Tolerance" config that stops the motor automatically
        // for PositionVoltage,
        // it just servos to the position.

        // Forward Limit: 270 degrees.
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (TurretConstants.MaxAngle / 360.0)
                / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // Reverse Limit: -90 degrees.
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (TurretConstants.MinAngle / 360.0)
                / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        // Safety: Current Limit
        turretConfig.CurrentLimits.StatorCurrentLimit = TurretConstants.CurrentLimit;
        turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        TurretMotor.getConfigurator().apply(turretConfig);

        TurretMotor.setPosition(0);

        DriveTrain = drivetrain;
    }

    @Override
    public void periodic() {
        if (DriveTrain == null || zoneDetection == null) {
            return;
        }
        // --- 1. Sensors & State ---
        // Get current position in Rotations
        double currentMotorRotations = TurretMotor.getPosition().getValueAsDouble();
        // Convert to Degrees for Logic
        double currentTurretDegrees = currentMotorRotations *
                TurretConstants.TurretGearRatio * 360.0;

        double robotHeadingDegrees = (DriveTrain != null) ? DriveTrain.getState().Pose.getRotation().getDegrees() : 0.0;
        double currentTurretFieldDegrees = robotHeadingDegrees +
                currentTurretDegrees; // Approximate field heading

        SmartDashboard.putNumber("Turret/Field Angle", currentTurretFieldDegrees);
        SmartDashboard.putNumber("Turret/Relative Angle", currentTurretDegrees);

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

        // --- 3. Calculate Desired Angle ---
        double targetRelativeDegrees = 0.0;

        if (shouldTrack && targetPose != null) {
            Pose2d currentRobotPose = DriveTrain.getState().Pose;
            ChassisSpeeds speeds = DriveTrain.getState().Speeds;

            targetRelativeDegrees = ShootingKinematics.getLeadAdjustedAngleDegrees(
                    currentRobotPose,
                    speeds,
                    targetPose.getTranslation(),
                    Constants.ShootingConstants.BallSpeedMps,
                    Constants.ShootingConstants.LeadAngleScale);

            SmartDashboard.putNumber("Turret/Target Field Angle",
                    robotHeadingDegrees + targetRelativeDegrees);
        } else {
            // Idle / Forward
            targetRelativeDegrees = 0.0;
            SmartDashboard.putNumber("Turret/Target Field Angle", robotHeadingDegrees);
        }

        SmartDashboard.putBoolean("Turret/Tracking", shouldTrack);
        SmartDashboard.putNumber("Turret/Target Relative", targetRelativeDegrees);

        // --- 4. Closed Loop Control ---

        // Optimize the target angle to fit within the valid range of the Turret (-90 to
        // 90 for initial testing)
        double constrainedTargetDegrees = MathUtil.clamp(targetRelativeDegrees, TurretConstants.MinAngle,
                TurretConstants.MaxAngle);

        SmartDashboard.putNumber("Turret/Target Constrained",
                constrainedTargetDegrees);
        SmartDashboard.putNumber("Turret/Error", constrainedTargetDegrees -
                currentTurretDegrees);

        // Convert Target Degrees -> Motor Rotations
        // Rotations = Degrees / 360
        // Motor Rotations = Turret Rotations / Gear Ratio
        double targetRotations = (constrainedTargetDegrees / 360.0) /
                TurretConstants.TurretGearRatio;

        // Apply to Motor
        TurretMotor.setControl(m_request.withPosition(targetRotations));
    }

    public void setPower(float speed) {
        TurretMotor.set(speed);
    }

    public void setTargetAngle(double targetDegrees) {
        double constrainedTargetDegrees = MathUtil.clamp(targetDegrees, TurretConstants.MinAngle,
                TurretConstants.MaxAngle);
        double targetRotations = (constrainedTargetDegrees / 360.0) /
                TurretConstants.TurretGearRatio;
        TurretMotor.setControl(m_request.withPosition(targetRotations));
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

    /** Returns true if turret is within tolerance of the target angle. */
    public boolean isOnTarget(double targetDegrees) {
        double current = GetCurrentAngle();
        return Math.abs(MathUtil.inputModulus(current - targetDegrees, -180, 180)) <= TurretConstants.kTolerance;
    }

    // Kept for reference or backward compatibility if other classes call it
    public double CalculateAngleToTarget() {
        // ... (This logic is now integrated into periodic, we can return dummy or
        // duplicate logic if needed)
        // For now returning 0 to simplify, as periodic handles the real calculation.
        return 0.0;
    }
}
