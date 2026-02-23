package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.math.MathUtil;
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
    private SwerveDrivetrain DriveTrain;
    private ZoneDetection zoneDetection;
    
    // TalonFX Control Request
    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // y in inches: 159.1 = 4.0386 m
    // x in inches: 182.1 = 4.6228 m

    @SuppressWarnings("rawtypes")
    public Turret(SwerveDrivetrain drivetrain, ZoneDetection zoneDetection) {
        this.zoneDetection = zoneDetection;
        TurretMotor = new TalonFX(TurretConstants.TurretCanId);

        // Configure TalonFX
        var turretConfig = new com.ctre.phoenix6.configs.TalonFXConfiguration();
        
        // Slot 0 PID config
        turretConfig.Slot0.kP = TurretConstants.kP;
        turretConfig.Slot0.kI = TurretConstants.kI;
        turretConfig.Slot0.kD = TurretConstants.kD;
        // Note: kTolerance is handled by the closed loop logic manually or by just checking error, 
        // TalonFX doesn't have a "Tolerance" config that stops the motor automatically for PositionVoltage,
        // it just servos to the position.
        
        // Forward Limit: 270 degrees. 
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = (TurretConstants.MaxAngle / 360.0) / TurretConstants.TurretGearRatio;
        turretConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

        // Reverse Limit: -90 degrees.
        turretConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = (TurretConstants.MinAngle / 360.0) / TurretConstants.TurretGearRatio;
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
        // --- 1. Sensors & State ---
        // Get current position in Rotations
        double currentMotorRotations = TurretMotor.getPosition().getValueAsDouble();
        // Convert to Degrees for Logic
        double currentTurretDegrees = currentMotorRotations * TurretConstants.TurretGearRatio * 360.0;
        
        double robotHeadingDegrees = DriveTrain.getState().Pose.getRotation().getDegrees();
        double currentTurretFieldDegrees = robotHeadingDegrees + currentTurretDegrees; // Approximate field heading
        
        SmartDashboard.putNumber("Turret/Field Angle", currentTurretFieldDegrees);
        SmartDashboard.putNumber("Turret/Relative Angle", currentTurretDegrees);

        // --- 2. Determine Target Pose ---
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        Pose2d targetPose = null;
        boolean shouldTrack = false;
        
        if (alliance.isPresent()) {
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
                    if (DriveTrain.getState().Pose.getY() < Constants.FieldConstants.FieldWidth / 2.0) {
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
                    if (DriveTrain.getState().Pose.getY() < Constants.FieldConstants.FieldWidth / 2.0) {
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
            Translation2d delta = targetPose.getTranslation().minus(DriveTrain.getState().Pose.getTranslation());
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
        
        // Optimize the target angle to fit within the valid range of the Turret (-90 to 270)
        // Since the turret has a 360 degree total range (but hard stops), there is only ONE valid physical angle
        // for any given direction. We force the target into that range manually.
        double constrainedTargetDegrees = MathUtil.inputModulus(targetRelativeDegrees, TurretConstants.MinAngle, TurretConstants.MaxAngle);
        
        SmartDashboard.putNumber("Turret/Target Constrained", constrainedTargetDegrees);
        SmartDashboard.putNumber("Turret/Error", constrainedTargetDegrees - currentTurretDegrees);

        // Convert Target Degrees -> Motor Rotations
        // Rotations = Degrees / 360
        // Motor Rotations = Turret Rotations / Gear Ratio
        double targetRotations = (constrainedTargetDegrees / 360.0) / TurretConstants.TurretGearRatio;
        
        // Apply to Motor
        TurretMotor.setControl(m_request.withPosition(targetRotations));
    }

    public void setPower(float speed) {
        TurretMotor.set(speed);
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
        // ... (This logic is now integrated into periodic, we can return dummy or duplicate logic if needed)
        // For now returning 0 to simplify, as periodic handles the real calculation.
        return 0.0;
    }
}
