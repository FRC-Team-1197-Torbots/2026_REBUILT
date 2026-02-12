package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Shooter extends SubsystemBase {
    private final int ShooterCANID1 = 21;
    private final int ShooterCANID2 = 22;
    private final int HoodCANDID = 23;
    
    private TalonFX shooterWheel1, shooterWheel2;
    private SparkFlex hood;
    private SparkClosedLoopController hoodController;
    private CommandXboxController m_controller;

    private double shooterspeed = 0.5f, hoodangle;
    private double ratio = (1d / 18d) * 360d;

    public Shooter(CommandXboxController controller) {
        shooterWheel1 = new TalonFX(ShooterCANID1);
        shooterWheel2 = new TalonFX(ShooterCANID2);

        hood = new SparkFlex(HoodCANDID, MotorType.kBrushless);
        hoodController = hood.getClosedLoopController();

        m_controller = controller;

        hood.getEncoder().setPosition(0);

        SmartDashboard.putNumber("Shooter Speed", 0);
        SmartDashboard.putNumber("Hood Angle", 0);
    }

    public void Spin() {
        shooterWheel1.set(shooterspeed);
        shooterWheel2.set(shooterspeed);
    }

    public void Stop() {
        shooterWheel1.set(0);
        shooterWheel2.set(0);
    }

    public Command runShooterCommand() {
        return run(this::Spin).finallyDo(interrupted -> Stop());
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
