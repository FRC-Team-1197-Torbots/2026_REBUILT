package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
    public TalonFX climbMotor;
    public CANrange range; 

    private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    public Climber() {
        climbMotor = new TalonFX(Constants.ClimberConstants.ClimberCanId);
        range = new CANrange(Constants.ClimberConstants.ClimberDateId);

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = Constants.ClimberConstants.kP;
        slot0Configs.kI = Constants.ClimberConstants.kI;
        slot0Configs.kD = Constants.ClimberConstants.kD;

        // Safety: Soft Limits
        var climbConfig = new TalonFXConfiguration();
        climbConfig.Slot0 = slot0Configs; // Re-use the slot0 configs we just made
        
        // Max Extension: L3 + small buffer
        climbConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = metersToRotations(Constants.ClimberConstants.L3 + Constants.ClimberConstants.SoftLimitBuffer); 
        climbConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        
        // Min Retraction: 0
        climbConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = metersToRotations(0.0);
        climbConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

        climbMotor.getConfigurator().apply(climbConfig);
        climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    @Override
    public void periodic() {
        super.periodic();
        SmartDashboard.putNumber("Climber Height (m)", getHeight());
        SmartDashboard.putNumber("Climber Range", getRange());
    }

    public double metersToRotations(double meters) {
        double circumference = Math.PI * Constants.ClimberConstants.ClimberSpoolDiameter;
        double rotationsAtSpool = meters / circumference;
        return rotationsAtSpool * Constants.ClimberConstants.ClimberGearRatio;
    }

    public double rotationsToMeters(double rotations) {
        double circumference = Math.PI * Constants.ClimberConstants.ClimberSpoolDiameter;
        double rotationsAtSpool = rotations / Constants.ClimberConstants.ClimberGearRatio;
        return rotationsAtSpool * circumference;
    }

    public void moveToPosition(double meters) {
        double rotations = metersToRotations(meters);
        climbMotor.setControl(m_request.withPosition(rotations));
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    public double getHeight() {
        return rotationsToMeters(climbMotor.getPosition().getValueAsDouble());
    }

    public double getRange() {
        return range.getDistance().getValueAsDouble();
    }

    public Command getAlignToClimbCommand() {
        Pose2d targetClimbPose = Constants.ClimberConstants.lowerClimbPosition;

        PathConstraints constraints = new PathConstraints(3.5,
            3.5, Units.degreesToRadians(540),
            Units.degreesToRadians(360));

        return AutoBuilder.pathfindToPose(targetClimbPose, constraints, 0.0);
    }

    /** Returns true when climber is within Tolerance of the target height. */
    public boolean atHeight(double targetMeters) {
        return Math.abs(getHeight() - targetMeters) < Constants.ClimberConstants.Tolerance;
    }

    /** Command to climb one level (L1). */
    public Command climbOneLevelCommand() {
        return run(() -> moveToPosition(Constants.ClimberConstants.L1))
                .until(() -> atHeight(Constants.ClimberConstants.L1));
    }

    /** Command to climb three levels (L3). */
    public Command climbThreeLevelsCommand() {
        return run(() -> moveToPosition(Constants.ClimberConstants.L3))
                .until(() -> atHeight(Constants.ClimberConstants.L3));
    }

    /** Command to retract climber to ground (L0). */
    public Command retractCommand() {
        return run(() -> moveToPosition(Constants.ClimberConstants.L0))
                .until(() -> atHeight(Constants.ClimberConstants.L0));
    }

    /** Command to stop the climber motor. */
    public Command stopCommand() {
        return runOnce(this::stop);
    }
}
