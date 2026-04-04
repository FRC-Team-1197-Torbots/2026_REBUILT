package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private SparkMax shooterWheel1, shooterWheel2;
    private PIDController m_pidController;
    private final edu.wpi.first.math.controller.SimpleMotorFeedforward m_feedforward;

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
        // config1.smartCurrentLimit(70);

        // Invert the main motor if it's the left side
        config1.inverted(m_side == SHOOTER_SIDE.LEFT);

        m_pidController = new PIDController(Constants.ShooterConstants.kP, 0, 0);
        // kV = 12 Volts / Max RPM. Assuming ~5600 max RPM for NEOs/Vortex on flywheels.
        // Adjust Constants.ShooterConstants.kV accordingly (e.g., 0.0021).
        m_feedforward = new edu.wpi.first.math.controller.SimpleMotorFeedforward(0.0, Constants.ShooterConstants.kV);
        // Basic PID configuration (Needs to be tuned)
        // config1.closedLoop.pid(0.008, 0, 0);

        // Velocity filtering fix for Flywheels (From Chief Delphi)
        // Reduces phase lag from default 164ms down to ~5ms
        // config1.encoder.uvwMeasurementPeriod(32).uvwAverageDepth(8);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.apply(config1); // Inherit all limits, coastal, inverted state, etc.
        config2.follow(shooterWheel1, true); // Follow shooterWheel1 but inverted

        shooterWheel1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterWheel2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void Spin(double speedMeasurement) {
        // SparkMax expects RPM, so if RPS is given, convert to RPM
        shooterspeed = speedMeasurement;
        double rpm = speedMeasurement * 60.0;
        
        // Feedforward does 90% of the work, PID just cleans up the error
        double ffVoltage = m_feedforward.calculate(rpm);
        double pidVoltage = m_pidController.calculate(shooterWheel1.getEncoder().getVelocity(), rpm);

        double totalVoltage = ffVoltage + pidVoltage;

        // Optionally clamp total voltage to realistic battery limits (12V)
        totalVoltage = MathUtil.clamp(totalVoltage, 0.0, 12.0);

        shooterWheel1.setVoltage(totalVoltage);
    }

    public void runIdle() {
        Spin(Constants.ShooterConstants.IdleSpeed);
    }

    public void Shoot() {
        Spin(shooterspeed);
    }

    public void setShooterSpeed(double speed) {
        shooterspeed = speed;
    }

    public void Stop() {
        // Idle speed is now technically a target RPM
        Spin(ShooterConstants.IdleSpeed); // Keep it spinning at idle instead of full stop
    }



    public boolean isAtSpeed() {
        // SparkMax returns velocity in RPM
        double currentSpeedRpm = shooterWheel1.getEncoder().getVelocity();
        double currentSpeedRps = currentSpeedRpm / 60.0;
        return Math.abs(currentSpeedRps - (shooterspeed)) <=  1.0;
    }

    @Override
    public void periodic() {
        super.periodic();

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel1 Current", shooterWheel1.getOutputCurrent());
        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel2 Current", shooterWheel2.getOutputCurrent());

        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel1 Speed", shooterWheel1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel2 Speed", shooterWheel2.getEncoder().getVelocity());

        // SmartDashboard.putNumber("Shooter " + m_side.name() + "/Wheel1 Speed",
        //         shooterWheel1.getEncoder().getVelocity()/60.0);
        // SmartDashboard.putBoolean("Shooter " + m_side.name() + "/Is At Speed",
        //         isAtSpeed());
        // SmartDashboard.putNumber("Shooter " + m_side.name() + "/Speed Request",
        //         shooterspeed);
    }
}
