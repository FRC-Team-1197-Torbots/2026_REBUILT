package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {

    private TalonFX TurretMotor;
    private SwerveDrivetrain DriveTrain;
    private double ratio = 24.0f / 240.0f;
    public double target;
    public double p;

    // y in inches: 159.1 = 4.0386 m
    // x in inches: 182.1 = 4.6228 m

    private Pose2d RedTarget = new Pose2d(4.0386, 4.6228, Rotation2d.kZero);

    @SuppressWarnings("rawtypes")
    public Turret(SwerveDrivetrain drivetrain) {
        TurretMotor = new TalonFX(TurretConstants.TurretCanId);
        TurretMotor.setPosition(0);

        DriveTrain = drivetrain;
    }

    @Override
    public void periodic() {
        //Convert the kracken encoder ticks to degrees
        double turretangle = MathUtil.inputModulus(TurretMotor.getPosition().getValueAsDouble() * ratio * 360.0f, -180d, 180d);             
        double robotAngle = DriveTrain.getState().Pose.getRotation().getDegrees();

        //combine the robot rotation and the turrets rotation to get a heading 
        //TODO :: Need to consider if the rotation exceeds 360 or 180
        double fieldTurretAngle = robotAngle + turretangle;
        SmartDashboard.putNumber("Field Turret Angle", fieldTurretAngle);
        
        //Calculate the field space delta in turret angle
        double angleToTarget = CalculateAngleToTarget();
        SmartDashboard.putNumber("Angle to Target", angleToTarget);

        //Calculate error for turret
        double err = angleToTarget - fieldTurretAngle;
        err = MathUtil.inputModulus(err, -180d, 180d);
        SmartDashboard.putNumber("Turret Error", err);

        //double power = err * p;          
        //TurretMotor.set(power);
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
        return TurretMotor.getPosition().getValueAsDouble() * ratio * 360.0f;
    }

    public double CalculateAngleToTarget() {
        Translation2d deltaVector = RedTarget.getTranslation().minus(DriveTrain.getState().Pose.getTranslation());

        Rotation2d angleToTarget = deltaVector.getAngle();

        return angleToTarget.getDegrees();
    }
}
