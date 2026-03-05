package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
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
    private final PositionVoltage m_DeployRequest = new PositionVoltage(0).withSlot(0);
    private final com.ctre.phoenix6.controls.NeutralOut m_StopDeployRequest = new com.ctre.phoenix6.controls.NeutralOut();

    private Double m_deployTarget = null;

    public Intake() {
        intakeMotor = new TalonFX(IntakeConstants.IntakeCanId);
        deployMotor = new TalonFX(IntakeConstants.IntakeDeployCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kP = IntakeConstants.kP;
        config.Slot0.kI = IntakeConstants.kI;
        config.Slot0.kD = IntakeConstants.kD;
        config.Slot0.kV = IntakeConstants.kV;
        
        // Safety: Current Limit for Intake Roller
        config.CurrentLimits.StatorCurrentLimit = IntakeConstants.RollerCurrentLimit;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(config);

        // Deploy Motor Config
        TalonFXConfiguration deployConfig = new TalonFXConfiguration();
        deployConfig.Slot0.kP = IntakeConstants.kDeployP;
        deployConfig.Slot0.kI = IntakeConstants.kDeployI;
        deployConfig.Slot0.kD = IntakeConstants.kDeployD;
        
        // Safety: Current Limit to prevent burnout on jam
        deployConfig.CurrentLimits.StatorCurrentLimit = IntakeConstants.DeployCurrentLimit; // Amps
        deployConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        deployMotor.getConfigurator().apply(deployConfig);
        deployMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
        deployMotor.setPosition(0); // Assume starting at Retracted (0)

        //SmartDashboard.setDefaultBoolean("Intake/UseVariableSpeed", true);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_deployTarget != null) {
            double currentPos = deployMotor.getPosition().getValueAsDouble();
            if (Math.abs(currentPos - m_deployTarget) < IntakeConstants.DeployTolerance) {
                SmartDashboard.putBoolean("Intake/Loop Running", false);
                stopDeploy();
            }
        }

        
        SmartDashboard.putNumber("Intake/Intake Position", deployMotor.getPosition().getValueAsDouble());
    }

    public void setSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        setSpeed(0.0);
    }

    public void runIntake(ChassisSpeeds speed) {
            
            double robotVelocity = Math.hypot(speed.vxMetersPerSecond, speed.vyMetersPerSecond);

            // Calculate target speed in Meters Per Second
            // Start at Min_Surface_Speed, bump up based on robot velocity
            double targetSpeed = Math.max(
                Constants.IntakeConstants.Min_Surface_Speed, 
                robotVelocity * Constants.IntakeConstants.RobotSpeedMultiplier
            );

            setSurfaceSpeed(targetSpeed);
    }

    public void runOuttake() {
        setSpeed(IntakeConstants.OuttakeSpeed);
    }

    public Command runIntakeCommand(java.util.function.Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get()))
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stopIntake());
    }

    public Command runIntakeCommand(ChassisSpeeds speed) {
        return run(() -> runIntake(speed))
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stopIntake());
    }

    public Command runOuttakeCommand() {
        // Assume we want to deploy to outtake as well, to clear the robot frame
        return run(() -> runOuttake())
            .beforeStarting(this::deploy)
            .finallyDo(interrupted -> stopIntake());
    }

    public Command stopCommand() {
        return runOnce(this::stopIntake);
    }
    
    // Deployment Methods
    public void deploy() {
        SmartDashboard.putBoolean("Intake/Loop Running", true);
        m_deployTarget = IntakeConstants.DeployPosition;
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.DeployPosition));
    }

    public void retract() {
        SmartDashboard.putBoolean("Intake/Loop Running", true);
        m_deployTarget = IntakeConstants.RetractPosition;
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.RetractPosition));
    }

    public Command runRetractCommand() {
        return runOnce(this::retract);
    }

    public Command runDeployCommand() {
        return runOnce(this::deploy);
    }
    
    // Agitation: Helps push balls towards shooter / unjam
    // Alternates between Forward and Reverse to clear jams
    public Command runAgitateCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.sequence(
            run(() -> setSpeed(-IntakeConstants.IntakeDutyCycle)).withTimeout(0.25), // Reverse for 0.25s
            run(() -> setSpeed(IntakeConstants.IntakeDutyCycle)).withTimeout(0.25)   // Forward for 0.25s
        ).repeatedly()
        .finallyDo(interrupted -> stopDeploy());
    }

    public void stopDeploy() {
        m_deployTarget = null;
        deployMotor.setControl(m_StopDeployRequest);
    }

    public void setSurfaceSpeed(double mps) {
        double wheelCircumferenceMeters = Units.inchesToMeters(Constants.IntakeConstants.wheelDiameter) * Math.PI;

        double targetRPS = (mps / wheelCircumferenceMeters) * Constants.IntakeConstants.gearratio;

        intakeMotor.setControl(m_VelocityRequest.withVelocity(targetRPS));
    }
}