package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {
    private TalonFX shooterWheel1, shooterWheel2;

    // Closed-loop control request
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private double shooterspeed = 60.0f; // Default to 60 RPS (3600 RPM)

    public Shooter() {
        shooterWheel1 = new TalonFX(ShooterConstants.ShooterCanId1);
        shooterWheel2 = new TalonFX(ShooterConstants.ShooterCanId2);

        // Configure PID and Feedforward for Slot 0
        TalonFXConfiguration configs = new TalonFXConfiguration();
        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = ShooterConstants.kP;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = ShooterConstants.kV;
        
        // Safety: Current Limit (High for Shooter)
        configs.CurrentLimits.StatorCurrentLimit = ShooterConstants.CurrentLimit;
        configs.CurrentLimits.StatorCurrentLimitEnable = true;

        shooterWheel1.getConfigurator().apply(configs);
        shooterWheel2.getConfigurator().apply(configs);

        SmartDashboard.putNumber("Shooter Speed", 60.0);
    }

    public void Spin() {
        // Use VelocityVoltage control
        shooterWheel1.setControl(m_request.withVelocity(shooterspeed));
        shooterWheel2.setControl(m_request.withVelocity(shooterspeed));
    }

    public void Stop() {
        shooterWheel1.stopMotor();
        shooterWheel2.stopMotor();
    }

    public Command runShooterCommand() {
        return run(this::Spin).finallyDo(interrupted -> Stop());
    }

    public Command runIdleCommand() {
        return run(() -> {
            shooterWheel1.setControl(m_request.withVelocity(Constants.ShooterConstants.IdleSpeed));
            shooterWheel2.setControl(m_request.withVelocity(Constants.ShooterConstants.IdleSpeed));
        });
    }

    public boolean isAtSpeed() {
        // Check if actual speed is within 5% of target speed
        double currentSpeed = shooterWheel1.getVelocity().getValueAsDouble();
        return Math.abs(currentSpeed - shooterspeed) <= (Math.abs(shooterspeed) * 0.05);
    }

    @Override
    public void periodic() {
        super.periodic();

        shooterspeed = SmartDashboard.getNumber("Shooter Speed", 0);
        
        SmartDashboard.putNumber("Actual Shooter Speed", shooterWheel1.getVelocity().getValueAsDouble());
    }
}
