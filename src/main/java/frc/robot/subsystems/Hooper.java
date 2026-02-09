package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.HopperConstants;

public class Hooper extends SubsystemBase {
    private final SparkFlex hopperMotor1;
    private final SparkFlex hopperMotor2;
    private final TalonFX hopperSplitterMotor;

    private double hopperSpeed = 0.0;

    public Hooper() {
        hopperMotor1 = new SparkFlex(HopperConstants.HopperCanId1, SparkLowLevel.MotorType.kBrushless);
        hopperMotor2 = new SparkFlex(HopperConstants.HopperCanId2, SparkLowLevel.MotorType.kBrushless);
        hopperSplitterMotor = new TalonFX(HopperConstants.HooperSplitterCanID, "rio");
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Hopper Set Speed", hopperSpeed);
        SmartDashboard.putNumber("Hopper Motor 1 Output", hopperMotor1.getAppliedOutput());
        SmartDashboard.putNumber("Hopper Motor 2 Output", hopperMotor2.getAppliedOutput());
    }

    /** Sets both hopper motors to the same speed. */
    public void setSpeed(double speed) {
        hopperSpeed = speed;
        hopperMotor1.set(speed);
        hopperMotor2.set(-speed);
        hopperSplitterMotor.set(speed);
    }

    public void stop() {
        setSpeed(0.0);
    }

    public void feed(double speed) {
        setSpeed(Math.abs(speed));
    }

    public void reverse(double speed) {
        setSpeed(-Math.abs(speed));
    }

    public Command runHopper(double speed) {
        return run(() -> setSpeed(speed));
    }

    /** Runs both hoppers at default feed speed; stops when command ends (e.g. button released). */
    public Command runHopperCommand() {
        return run(() -> feed(HopperConstants.HopperFeedSpeed)).finallyDo(interrupted -> stop());
    }

    public Command feedCommand(double speed) {
        return run(() -> feed(speed));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
