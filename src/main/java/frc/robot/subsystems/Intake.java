package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import java.util.function.Supplier;
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

    public enum INTAKE_POSITION {
        DEPLOYED, RETRACTED, AGI, RETRACTING
    };

    public INTAKE_POSITION m_position;

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
        deployMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        deployMotor.setPosition(0); // Assume starting at Retracted (0)

        m_position = INTAKE_POSITION.RETRACTED;
    }

    @Override
    public void periodic() {
        super.periodic();

        // SmartDashboard.putNumber("Intake/Intake Position", deployMotor.getPosition().getValueAsDouble());

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
                // If it stalls at either end, assume it hit the physical hard stop and reset the encoder
                // This ensures repeated deployments remain accurate even if the mechanism skips teeth.
                if (isStalled) {
                    if (m_deployTarget == IntakeConstants.DeployPosition) {
                        deployMotor.setPosition(IntakeConstants.DeployPosition);
                    } else if (m_deployTarget == IntakeConstants.RetractPosition) {
                        deployMotor.setPosition(IntakeConstants.RetractPosition);
                    }
                }

                stopDeploy();
            }
        }
    }

    public boolean isRollerLocked() {
        return m_position == INTAKE_POSITION.AGI || (m_deployTarget != null && m_deployTarget == IntakeConstants.AgiPosition);
    }

    public void setSpeed(double speed) {
        if (isRollerLocked()) {
            intakeMotor.set(0.0);
        } else {
            intakeMotor.set(speed);
        }
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



    public Command stopCommand() {
        return runOnce(this::stopIntake);
    }

    public void agi() {
        if (m_position == INTAKE_POSITION.AGI)
            return;

        m_deployTarget = IntakeConstants.AgiPosition;
        m_deployTimer.restart();
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.AgiPosition));
        stopIntake();
    }

    // Deployment Methods
    public void deploy() {
        // if (m_position == INTAKE_POSITION.DEPLOYED)
        // return;

        m_deployTarget = IntakeConstants.DeployPosition;
        m_deployTimer.restart();
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.DeployPosition));
    }

    public void retract() {
        stopIntake();

        if (m_position == INTAKE_POSITION.RETRACTED)
            return;

        m_position = INTAKE_POSITION.RETRACTED;

        m_deployTarget = IntakeConstants.RetractPosition;
        m_deployTimer.restart();
        deployMotor.setControl(m_DeployRequest.withPosition(IntakeConstants.RetractPosition));
    }

    public void stopDeploy() {
        if (m_deployTarget == IntakeConstants.DeployPosition)
            m_position = INTAKE_POSITION.DEPLOYED;

        if (m_deployTarget == IntakeConstants.RetractPosition)
            m_position = INTAKE_POSITION.RETRACTED;

        m_deployTarget = null;
        deployMotor.setControl(m_StopDeployRequest);
    }

    public Command runRetractCommand() {
        return runOnce(this::retract);
    }

    public Command runDeployCommand() {
        return runOnce(this::deploy);
    }

    public Command runAgiCommand() {
        return runOnce(this::agi);
    }


    /**
     * Deploys the intake and runs the rollers.
     * When the command ends (e.g., button released), the rollers stop and the
     * intake retracts.
     */
    public Command runDeployAndIntakeCommand(java.util.function.Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get())) // Run intake rollers indefinitely
                .beforeStarting(this::deploy);
    }

    public Command runDeployImmediate(Supplier<ChassisSpeeds> speedSupplier) {
        return runOnce(() -> runIntake(speedSupplier.get())) // Run intake rollers indefinitely
                .beforeStarting(this::deploy);
    }

    public Command runIntakeWheelAuto1(Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get())).withTimeout(4.7)
                .beforeStarting(this::deploy);
    }

        public Command runIntakeWheelAuto2(Supplier<ChassisSpeeds> speedSupplier) {
        return run(() -> runIntake(speedSupplier.get())).withTimeout(6)
                .beforeStarting(this::deploy);
    }

    public Command deployIntake() {
        return run(() -> deploy());
    }

    public void setSurfaceSpeed(double mps) {
        if (isRollerLocked()) {
            intakeMotor.set(0.0);
            return;
        }

        double wheelCircumferenceMeters = Units.inchesToMeters(Constants.IntakeConstants.wheelDiameter) * Math.PI;

        double targetRPS = (mps / wheelCircumferenceMeters) * Constants.IntakeConstants.gearratio;

        intakeMotor.setControl(m_VelocityRequest.withVelocity(targetRPS));
    }
}