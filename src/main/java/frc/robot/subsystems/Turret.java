package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;

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

    private double PDTest;

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

    public Turret(int turretCanId, int encoderID, edu.wpi.first.math.geometry.Translation2d turretOffset,
            SwerveDrivetrain<?, ?, ?> drivetrain, ZoneDetection zoneDetection, TURRENT_SIDE side) {

        this.zoneDetection = zoneDetection;
        m_robotOffset = turretOffset;
        m_side = side;
        DriveTrain = drivetrain;

        encoder = new CANcoder(encoderID);
        TurretMotor = new TalonFX(turretCanId);

        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();

        TurretMotor.getConfigurator().apply(turretConfig);

        encoder.setPosition(0);

        turrentPID = new PIDController(Constants.TurretConstants.kP, Constants.TurretConstants.kI,
                Constants.TurretConstants.kD);
        turrentPID.disableContinuousInput();
        SmartDashboard.putNumber("Turret/PD", 0);
        SmartDashboard.putNumber("Turret/ErrorPercent", 0);
    }

    @Override
    public void periodic() {

        PDTest = SmartDashboard.getNumber("Turret/PD", 0);
        turrentPID.setPID(PDTest, 0, 0);

        if (m_side == TURRENT_SIDE.LEFT) {
            SmartDashboard.putNumber("Left Encoder", encoder.getPosition().getValueAsDouble());
        }

        boolean usePosition = SmartDashboard.getBoolean("Turret/UseManualPosition", false);
        if (usePosition) {
            double leftAngle = SmartDashboard.getNumber("Turret/ManualAngleLeft", 0.0);
            //double rightAngle = SmartDashboard.getNumber("Turret/ManualAngleRight", 0.0);
            setTargetAngle(leftAngle);
            // rightTurret.setTargetAngle(rightAngle);
        } else {
            double leftPower = SmartDashboard.getNumber("Turret/ManualPowerLeft", 0.0);
            //double rightPower = SmartDashboard.getNumber("Turret/ManualPowerRight", 0.0);
            setPower((float) leftPower);
            // rightTurret.setPower((float) rightPower);
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
            Pose2d turretFieldPose = currentRobotPose.transformBy(
                    new edu.wpi.first.math.geometry.Transform2d(m_robotOffset, new Rotation2d()));

            Translation2d delta = targetPose.getTranslation().minus(turretFieldPose.getTranslation());
            double targetFieldDegrees = delta.getAngle().getDegrees();

            SmartDashboard.putNumber("Turret/Target Field Angle", targetFieldDegrees);

            // RobotHeading + TurretRelative = TargetField
            // TurretRelative = TargetField - RobotHeading
            targetRelativeDegrees = targetFieldDegrees - robotHeadingDegrees;
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
        getErrorInPer();

        // Convert Target Degrees -> Motor Rotations
        // Rotations = Degrees / 360
        // Motor Rotations = Turret Rotations / Gear Ratio
        double targetRotations = (constrainedTargetDegrees / 360.0) /
                TurretConstants.TurretGearRatio;

        // Apply to Motor
        // TurretMotor.setControl(m_request.withPosition(targetRotations));
    }

    public void setPower(float speed) {
        TurretMotor.set(speed);
    }

    public void setTargetAngle(double targetRotations) {
        double motoroutput = turrentPID.calculate(encoder.getPosition().getValueAsDouble(), targetRotations);

        motoroutput = MathUtil.clamp(motoroutput, -0.75, 75);

        SmartDashboard.putNumber("PID Result", motoroutput);

        TurretMotor.set(motoroutput);
    }

    public void getErrorInPer() {

        double actualPosition = encoder.getPosition().getValueAsDouble();
        double targetPosition = turrentPID.getSetpoint();

        double error = Math.abs(targetPosition - actualPosition);

        double percentError = 0;

        if (Math.abs(targetPosition) > 0.0001) { // prevent divide by zero
            percentError = (error / Math.abs(targetPosition)) * 100.0;
        }

        SmartDashboard.putNumber("Turret/PercentError", percentError);
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

    // Kept for reference or backward compatibility if other classes call it
    public double CalculateAngleToTarget() {
        // ... (This logic is now integrated into periodic, we can return dummy or
        // duplicate logic if needed)
        // For now returning 0 to simplify, as periodic handles the real calculation.
        return 0.0;
    }
}
