package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

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

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;
        config.Slot0.kV = IntakeConstants.kV;

        intakeMotor.getConfigurator().apply(config);

        SmartDashboard.setDefaultBoolean("Intake/UseVariableSpeed", true);
    }

    @Override
    public void periodic() {
        SmartDashboard.getBoolean("Intake/UseVariableSpeed", true);
        super.periodic();
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void runIntake(ChassisSpeeds speed) {
        if (SmartDashboard.getBoolean("Intake/UseVariableSpeed", true)) {
            double robotVelocity = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

            // Calculate target speed in Meters Per Second
            // Start at Min_Surface_Speed, bump up based on robot velocity
            double targetSpeed = Math.max(
                Constants.IntakeConstants.Min_Surface_Speed, 
                robotVelocity * Constants.IntakeConstants.RobotSpeedMultiplier
            );

            setSurfaceSpeed(targetSpeed);
        } else {
            // Constant Speed: Runs at fixed Duty Cycle
            setSpeed(Constants.IntakeConstants.IntakeDutyCycle);
        }
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

        intakeMotor.setControl(m_VelocityRequest.withVelocity(targetRPS));
    }
}