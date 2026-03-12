package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private SparkMax shooterWheel1, shooterWheel2;
    private PIDController m_pidController;

    public enum SHOOTER_SIDE {
        RIGHT, LEFT
    };

    protected SHOOTER_SIDE m_side;

    private double shooterspeed = 60.0; // Default to 60 RPS

    public Shooter(int shooterID1, int shooterID2, SHOOTER_SIDE side) {

        m_side = side;
        shooterWheel1 = new SparkMax(shooterID1, MotorType.kBrushless);
        shooterWheel2 = new SparkMax(shooterID2, MotorType.kBrushless);

        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.idleMode(IdleMode.kCoast);

        // Invert the main motor if it's the left side
        config1.inverted(m_side == SHOOTER_SIDE.LEFT);

        m_pidController = new PIDController(Constants.ShooterConstants.kP, 0, 0);
        // Basic PID configuration (Needs to be tuned)
        //config1.closedLoop.pid(0.008, 0, 0);

        // Velocity filtering fix for Flywheels (From Chief Delphi)
        // Reduces phase lag from default 164ms down to ~5ms
        //config1.encoder.uvwMeasurementPeriod(32).uvwAverageDepth(8);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.apply(config1); // Inherit all limits, coastal, inverted state, etc.
        config2.follow(shooterWheel1, true); // Follow shooterWheel1 but inverted

        shooterWheel1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterWheel2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setSpeed(double rps) {
        shooterspeed = rps;
        double rpm = rps * 60.0;
        double power = m_pidController.calculate(shooterWheel1.getEncoder().getVelocity(), rpm);
        shooterWheel1.set(power);
    }

    public void Spin(double speedMeasurement) {
        // SparkMax expects RPM, so if RPS is given, convert to RPM
        shooterspeed = speedMeasurement;
        double rpm = speedMeasurement * 60.0;

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Speed requested",
                rpm);

        double power = m_pidController.calculate(shooterWheel1.getEncoder().getVelocity(), rpm);
        power = MathUtil.clamp(power, 0.2f, 1);
        shooterWheel1.setVoltage(power * 12);

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Power Requested", power);
    }

    public void Hack() {
        shooterWheel1.set(Constants.ShooterConstants.ShootSpeed);
    }

    public void runIdle() {
        shooterWheel1.set(Constants.ShooterConstants.IdleSpeed);
    }

    public void Spin() {
        Spin(shooterspeed);
    }

    public void Stop() {
        Spin(ShooterConstants.IdleSpeed); // Keep it spinning at idle instead of full stop
    }

    public Command runShooterCommand() {
        return run(this::Spin).finallyDo(interrupted -> Stop());
    }

    public boolean isAtSpeed() {
        // SparkMax returns velocity in RPM
        double currentSpeedRpm = shooterWheel1.getEncoder().getVelocity();
        double currentSpeedRps = currentSpeedRpm / 60.0;
        return Math.abs(currentSpeedRps - shooterspeed) <= (Math.abs(shooterspeed) * 0.05);
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel1 Speed",
                shooterWheel1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel2 Speed",
                shooterWheel2.getEncoder().getVelocity());
    }
}
