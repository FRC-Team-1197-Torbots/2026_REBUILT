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
    private final TalonFX deployMotor;
    
    private final VelocityVoltage m_VelocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage m_DeployRequest = new PositionVoltage(0).withSlot(0); // Slot 0 for Deploy PID

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.IntakeCanId, "rio");
        deployMotor = new TalonFX(IntakeConstants.IntakeDeployCanId, "rio");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;
        config.Slot0.kV = IntakeConstants.kV;
        intakeMotor.getConfigurator().apply(config);

        // Deploy Motor Config
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.Slot0.kP = IntakeConstants.kDeployP;
        deployConfig.Slot0.kI = IntakeConstants.kDeployI;
        deployConfig.Slot0.kD = IntakeConstants.kDeployD;
        deployMotor.getConfigurator().apply(deployConfig);
        deployMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        deployMotor.setPosition(0); // Assume starting at Retracted (0)

        SmartDashboard.setDefaultBoolean("Intake/UseVariableSpeed", true);
    }

    @Override
    public void periodic() {
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

    public Command runIntakeCommand(java.util.function.Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get()))
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stop());
    }

    public Command runIntakeCommand(ChassisSpeeds speed) {
        return run(() -> runIntake(speed))
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stop());
    }

    public Command runOuttakeCommand() {
        // Assume we want to deploy to outtake as well, to clear the robot frame
        return run(() -> runOuttake())
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stop());
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
    
    // Deployment Methods
    public void deploy() {
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.DeployPosition));
    }

    public void retract() {
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.RetractPosition));
    }

    public Command runRetractCommand() {
        return runOnce(this::retract);
    }
    
    // Agitation: Helps push balls towards shooter / unjam
    // Alternates between Forward and Reverse to clear jams
    public Command runAgitateCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.sequence(
            run(() -> setSpeed(-IntakeConstants.IntakeDutyCycle)).withTimeout(0.25), // Reverse for 0.25s
            run(() -> setSpeed(IntakeConstants.IntakeDutyCycle)).withTimeout(0.25)   // Forward for 0.25s
        ).repeatedly()
        .finallyDo(interrupted -> stop());
    }

    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    public void setSurfaceSpeed(double mps) {
        double wheelCircumferenceMeters = Units.inchesToMeters(Constants.IntakeConstants.wheelDiameter) * Math.PI;

        double targetRPS = (mps / wheelCircumferenceMeters) * Constants.IntakeConstants.gearratio;

        intakeMotor.setControl(m_VelocityRequest.withVelocity(targetRPS));
    }
}