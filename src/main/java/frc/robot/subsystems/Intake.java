package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final VelocityVoltage m_VelocityRequest = new VelocityVoltage(0).withSlot(0);

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.IntakeCanId, "rio");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
        super.periodic();
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void runIntake(ChassisSpeeds speed) {
        double robotVelocity = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

        double targetSpeed = Math.max(Constants.IntakeConstants.Min_Intake_Speed, robotVelocity * Constants.IntakeConstants.RobotSpeedMultiplier);

        setSpeed(Constants.IntakeConstants.Min_Intake_Speed);
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

    public void setSurfaceSpeed(double mps) {
        double wheelCircumferenceMeters = Units.inchesToMeters(Constants.IntakeConstants.wheelDiameter) * Math.PI;

        double targetRPS = (mps / wheelCircumferenceMeters) * Constants.IntakeConstants.gearratio;

        //intakeMotor.setControl()
    }
}