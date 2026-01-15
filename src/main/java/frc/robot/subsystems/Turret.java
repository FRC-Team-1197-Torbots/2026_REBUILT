package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {

    private final int TurretCANID = 20;
    private TalonFX TurrentMotor;
    private SwerveDrivetrain DriveTrain;
    private Pose2d turretPose;
    private double ratio = 24.0f / 240.0f;

    // y in inches: 159.1
    // x in inches: 182.1

    private Pose2d RedTarget = new Pose2d(TurretCANID, TurretCANID, null);

    public Turret(SwerveDrivetrain drivetrain) {
        TurrentMotor = new TalonFX(TurretCANID);
        TurrentMotor.setPosition(0);

        DriveTrain = drivetrain;
        turretPose = new Pose2d();
    }

    /*public Command TrackDefaultCommand() {
        if (turretPose != null) {
            //calculate the vector between the robot and the target

            //calculate teh angle between the turret and the 
        } else {
            System.err.println("Turret Pose is null");
        }
    }*/

    @Override
    public void periodic() {
        SwerveDriveState state = DriveTrain.getState();
        double turretangle = TurrentMotor.getPosition().getValueAsDouble() * ratio * 360.0f;
        SmartDashboard.putNumber("Turret Angle", turretangle);

        turretPose = new Pose2d(state.Pose.getX(), state.Pose.getY(), new Rotation2d(turretangle));
    }

    public void setPower(float speed) {
        runOnce(() -> TurrentMotor.set(speed));
    }

    public Command ManualTurnLeft() {
        return runOnce(() -> TurrentMotor.set(0.1f));
    }

    public Command ManualTurnRight() {
        return runOnce(() -> TurrentMotor.set(-0.1f));
    }

    public Command StopTurret() {
        return runOnce(() -> TurrentMotor.set(0));
    }

    public double GetCurrentAngle() {
        return TurrentMotor.getPosition().getValueAsDouble() * ratio * 360.0f;
    }
}
