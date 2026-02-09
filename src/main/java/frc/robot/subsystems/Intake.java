package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private double intakeSpeed = 0.0;

    private double intakeTestSpeed = 0;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.IntakeCanId, "rio");
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void setSpeed(double speed) {
        intakeSpeed = speed;
        intakeMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void runIntake(ChassisSpeeds speed) {
        double robotVelocity = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

        double targetSpeed = Math.max(Constants.IntakeConstants.Min_Intake_Speed, robotVelocity * Constants.IntakeConstants.RobotSpeedMultiplier);

        setSpeed(targetSpeed);
    }

    public void runOuttake() {
        setSpeed(IntakeConstants.OuttakeSpeed);
    }

    public Command runIntakeCommand(ChassisSpeeds speed) {
        return run(() -> runIntake(speed)).finallyDo(interrupted -> stop());
    }

    public Command runOuttakeCommand() {
        return run(() -> runOuttake()).finallyDo(interrupted -> stop());
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}