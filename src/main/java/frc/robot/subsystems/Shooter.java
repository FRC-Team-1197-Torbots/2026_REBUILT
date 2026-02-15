package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private final int ShooterCANID1 = 21; //change to Neo 2.0
    private final int ShooterCANID2 = 22;
    private final int HoodCANDID = 23; //change to X44
    
    private TalonFX shooterWheel1, shooterWheel2;
    private SparkFlex hood;
    private SparkClosedLoopController hoodController;

    // Closed-loop control request
    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

    private double shooterspeed = 60.0f, hoodangle; // Default to 60 RPS (3600 RPM)
    private double ratio = (1d / 18d) * 360d;

    public Shooter() {
        shooterWheel1 = new TalonFX(ShooterCANID1);
        shooterWheel2 = new TalonFX(ShooterCANID2);

        // Configure PID and Feedforward for Slot 0
        TalonFXConfiguration configs = new TalonFXConfiguration();
        Slot0Configs slot0 = configs.Slot0;
        slot0.kP = 0.11;
        slot0.kI = 0.0;
        slot0.kD = 0.0;
        slot0.kV = 0.12;
        
        shooterWheel1.getConfigurator().apply(configs);
        shooterWheel2.getConfigurator().apply(configs);

        hood = new SparkFlex(HoodCANDID, MotorType.kBrushless);
        hoodController = hood.getClosedLoopController();

        hood.getEncoder().setPosition(0);

        SmartDashboard.putNumber("Shooter Speed", 60.0);
        SmartDashboard.putNumber("Hood Angle", 0);
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

    public void GoToAngle() {
        double ticks = ConvertDegreesToTicks(hoodangle);
        SmartDashboard.putNumber("Requested Ticks", ticks);

        hoodController.setSetpoint(ticks, ControlType.kPosition);
    }

    @Override
    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();

        shooterspeed = SmartDashboard.getNumber("Shooter Speed", 0);
        hoodangle = SmartDashboard.getNumber("Hood Angle", 0);       

        SmartDashboard.putNumber("Actual Shooter Speed", shooterWheel1.getVelocity().getValueAsDouble());
        SmartDashboard.putNumber("Actual Hood Angle", hood.getEncoder().getPosition() * ratio);     
    }

    private double ConvertTicksToAngle(double ticks) {
        return ticks * ratio;
    }

    private double ConvertDegreesToTicks(double angle) {
        return angle / ratio; 
    }
}
