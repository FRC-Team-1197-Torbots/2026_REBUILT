package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

@SuppressWarnings("removal")
public class Shooter extends SubsystemBase {
    private SparkMax shooterWheel1, shooterWheel2;
    private SparkClosedLoopController pidController1, pidController2;
    private Hood m_hood;
    private Turret m_turret;
    private Hopper m_hopper;

    private double shooterspeed = 60.0; // Default to 60 RPS

    public Shooter(int shooterID1, int shooterID2, Hopper hopper) {
        m_hopper = hopper;

        shooterWheel1 = new SparkMax(shooterID1, MotorType.kBrushless);
        shooterWheel2 = new SparkMax(shooterID2, MotorType.kBrushless);
    }

    public Shooter(int shooterID1, int shooterID2, int hoodID1, int hoodID2, int turretID,
            edu.wpi.first.math.geometry.Translation2d turretOffset, CommandSwerveDrivetrain drivetrain,
            ZoneDetection zoneDetection) {
        // m_hood = new Hood(hoodID1, hoodID2);
        // m_turret = new Turret(turretID, turretOffset, drivetrain, zoneDetection);

        shooterWheel1 = new SparkMax(shooterID1, MotorType.kBrushless);
        shooterWheel2 = new SparkMax(shooterID2, MotorType.kBrushless);

        pidController1 = shooterWheel1.getClosedLoopController();
        pidController2 = shooterWheel2.getClosedLoopController();

        SparkMaxConfig config1 = new SparkMaxConfig();
        // Configure PID constants from your ShooterConstants
        config1.closedLoop.pid(ShooterConstants.kP, 0, 0, ClosedLoopSlot.kSlot0);
        config1.closedLoop.velocityFF(ShooterConstants.kV, ClosedLoopSlot.kSlot0);
        config1.smartCurrentLimit((int) ShooterConstants.CurrentLimit);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.apply(config1); // Copy identical limits and PID

        // Invert the second wheel to spin the opposite way based on structural needs,
        // if required.
        // For Left/Right turrets where `LeftSpin` vs `RightSpin` applies polarity
        // inversions later, we simply initialize.

        shooterWheel1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterWheel2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SmartDashboard.putNumber("Shooter Speed", 60.0);
    }

    public void Spin(double speedMeasurement) {
        // SparkMax expects RPM, so if RPS is given, convert to RPM
        double rpm = speedMeasurement * 60.0;
        pidController1.setSetpoint(rpm, ControlType.kVelocity);
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

    /*
     * public Command runIdleCommand() {
     * return run(() -> DumbSpin(0.5));
     * }
     */

    public void RightSpin(double rpmSpeed) {
        // Right turret mapping: standard inversion setup as previously provided
        pidController1.setReference(rpmSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pidController2.setReference(-rpmSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("Right Shooter", shooterWheel1.getEncoder().getVelocity());
    }

    public void LeftSpin(double rpmSpeed) {
        // Left turret mapping: identical polarity
        pidController1.setReference(rpmSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pidController2.setReference(rpmSpeed, ControlType.kVelocity, ClosedLoopSlot.kSlot0);

        SmartDashboard.putNumber("Left Shooter", shooterWheel2.getEncoder().getVelocity());
    }

    public void setPowerLeft(double power) {
        shooterWheel1.set(power);
        shooterWheel2.set(power);
    }

    public void setPowerRight(double power) {
        shooterWheel1.set(power);
        shooterWheel2.set(-power);
    }

    public boolean isAtSpeed() {
        // SparkMax returns velocity in RPM
        double currentSpeedRpm = shooterWheel1.getEncoder().getVelocity();
        double currentSpeedRps = currentSpeedRpm / 60.0;
        return Math.abs(currentSpeedRps - shooterspeed) <= (Math.abs(shooterspeed) * 0.05);
    }

    /** Returns true if shooter is within 5% of target RPM. */
    public boolean isAtSpeed(double targetRpm) {
        double currentRpm = shooterWheel1.getEncoder().getVelocity();
        return Math.abs(currentRpm - targetRpm) <= Math.abs(targetRpm) * 0.05;
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

        SmartDashboard.putNumber("Shooter/Wheel1 Speed (RPM)", shooterWheel1.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Wheel2 Speed (RPM)", shooterWheel2.getEncoder().getVelocity());
    }
}
