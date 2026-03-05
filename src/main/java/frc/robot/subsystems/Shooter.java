package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private SparkMax shooterWheel1, shooterWheel2;
    private SparkClosedLoopController pidController1, pidController2;
    private Hood m_hood;
    private Turret m_turret;

    private double shooterspeed = 60.0; // Default to 60 RPS

    public Shooter(int shooterID1, int shooterID2, int hoodID1, int hoodID2, int turretID, edu.wpi.first.math.geometry.Translation2d turretOffset, CommandSwerveDrivetrain drivetrain, ZoneDetection zoneDetection) {
        m_hood = new Hood(hoodID1, hoodID2);
        m_turret = new Turret(turretID, turretOffset, drivetrain, zoneDetection);

        shooterWheel1 = new SparkMax(shooterID1, MotorType.kBrushless);
        shooterWheel2 = new SparkMax(shooterID2, MotorType.kBrushless);

        pidController1 = shooterWheel1.getClosedLoopController();
        pidController2 = shooterWheel2.getClosedLoopController();

        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.closedLoop.pid(ShooterConstants.kP, 0, 0, ClosedLoopSlot.kSlot0);
        config1.closedLoop.velocityFF(ShooterConstants.kV, ClosedLoopSlot.kSlot0);
        config1.smartCurrentLimit((int)ShooterConstants.CurrentLimit);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.apply(config1);
        config2.follow(shooterWheel1, true); // Follow the first motor, inverted

        shooterWheel1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterWheel2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter Speed", 60.0);
    }

    public void Spin(double speedMeasurement) {
        // SparkMax expects RPM, so if RPS is given, convert to RPM
        double rpm = speedMeasurement * 60.0;
        pidController1.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
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

    public Command runIdleCommand() {
        return run(() -> Spin(ShooterConstants.IdleSpeed));
    }

    public boolean isAtSpeed() {
        // SparkMax returns velocity in RPM
        double currentSpeedRpm = shooterWheel1.getEncoder().getVelocity();
        double currentSpeedRps = currentSpeedRpm / 60.0;
        return Math.abs(currentSpeedRps - shooterspeed) <= (Math.abs(shooterspeed) * 0.05);
    }

    public Hood getHood() {
        return m_hood;
    }

    public Turret getTurret() {
        return m_turret;
    }

    @Override
    public void periodic() {
        super.periodic();

        shooterspeed = SmartDashboard.getNumber("Shooter Speed", 60.0);
        
        //SmartDashboard.putNumber("Actual Shooter Speed", shooterWheel1.getEncoder().getVelocity() / 60.0);
    }
}
