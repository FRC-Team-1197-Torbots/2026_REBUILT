package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor;
    private double intakeSpeed = 0.0;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.IntakeCanId);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Intake Set Speed", intakeSpeed);
        SmartDashboard.putNumber("Intake Actual Output", intakeMotor.getDutyCycle().getValueAsDouble());
        SmartDashboard.putNumber("Intake Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    }

    public void setSpeed(double speed) {
        intakeSpeed = speed;
        intakeMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void runIntake() {
        setSpeed(IntakeConstants.IntakeSpeed);
    }

    public void runOuttake() {
        setSpeed(IntakeConstants.OuttakeSpeed);
    }

    public Command runIntakeCommand() {
        return run(() -> runIntake()).finallyDo(interrupted -> stop());
    }

    public Command runOuttakeCommand() {
        return run(() -> runOuttake()).finallyDo(interrupted -> stop());
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}