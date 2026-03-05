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
    private final edu.wpi.first.wpilibj.Timer m_deployTimer = new edu.wpi.first.wpilibj.Timer();

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

        // SmartDashboard.setDefaultBoolean("Intake/UseVariableSpeed", true);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (m_deployTarget != null) {
            double currentPos = deployMotor.getPosition().getValueAsDouble();
            double velocity = deployMotor.getVelocity().getValueAsDouble();
            double current = Math.abs(deployMotor.getStatorCurrent().getValueAsDouble());

            boolean isAtTarget = Math.abs(currentPos - m_deployTarget) < IntakeConstants.DeployTolerance;

            // If the motor is trying to move for at least 0.25s, but velocity is zero and
            // current is starting to spike,
            // Then it has likely reached the physical hard stop and is stalling.
            boolean isStalled = m_deployTimer.hasElapsed(0.25) &&
                    current > (IntakeConstants.DeployCurrentLimit - 10.0) &&
                    Math.abs(velocity) < 0.1;

            if (isAtTarget || isStalled) {
                SmartDashboard.putBoolean("Intake/Loop Running", false);
                SmartDashboard.putBoolean("Intake/Stalled", isStalled);

                // If it stalls while retracting, it hit the home position, so zero the encoder.
                // This ensures repeated deployments remain accurate even if skipping teeth.
                if (isStalled && m_deployTarget == IntakeConstants.RetractPosition) {
                    deployMotor.setPosition(0);
                }

                stopDeploy();
            } else {
                SmartDashboard.putBoolean("Intake/Stalled", false);
            }
        }

        SmartDashboard.putNumber("Intake/Intake Position", deployMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Intake/Deploy Current (Amps)",
                Math.abs(deployMotor.getStatorCurrent().getValueAsDouble()));
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
                robotVelocity * Constants.IntakeConstants.RobotSpeedMultiplier);

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
        m_deployTimer.restart();
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.DeployPosition));
    }

    public void retract() {
        SmartDashboard.putBoolean("Intake/Loop Running", true);
        m_deployTarget = IntakeConstants.RetractPosition;
        m_deployTimer.restart();
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.RetractPosition));
    }

    public void stopDeploy() {
        m_deployTarget = null;
        deployMotor.setControl(m_StopDeployRequest);
    }

    public Command runRetractCommand() {
        return runOnce(this::retract);
    }

    public Command runDeployCommand() {
        return runOnce(this::deploy);
    }

    /**
     * Deploys the intake and runs the rollers.
     * When the command ends (e.g., button released), the rollers stop and the
     * intake retracts.
     */
    public Command runDeployAndIntakeCommand(java.util.function.Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get())) // Run intake rollers indefinitely
                .beforeStarting(this::deploy) // Deploy when started
                .finallyDo(interrupted -> {
                    stopIntake(); // Stop rollers when finished
                    retract(); // Retract when finished
                });
    }

    // Agitation: Helps push balls towards shooter / unjam
    // Alternates between Forward and Reverse to clear jams
    public Command runAgitateCommand() {
        return edu.wpi.first.wpilibj2.command.Commands.sequence(
                run(() -> setSpeed(-IntakeConstants.IntakeDutyCycle)).withTimeout(0.25), // Reverse for 0.25s
                run(() -> setSpeed(IntakeConstants.IntakeDutyCycle)).withTimeout(0.25) // Forward for 0.25s
        ).repeatedly()
                .finallyDo(interrupted -> stopIntake()); // Should stop the intake, not deploy
    }

    public void setSurfaceSpeed(double mps) {
        double wheelCircumferenceMeters = Units.inchesToMeters(Constants.IntakeConstants.wheelDiameter) * Math.PI;

        double targetRPS = (mps / wheelCircumferenceMeters) * Constants.IntakeConstants.gearratio;

        intakeMotor.setControl(m_VelocityRequest.withVelocity(targetRPS));
    }
}