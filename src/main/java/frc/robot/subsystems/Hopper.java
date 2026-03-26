package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    // Motors
    private final TalonFX flopperMotor;
    private final TalonFX leftTower, rightTower;

    // Sensors
    private final CANrange canRange1;
    private final CANrange canRange2;

    private Intake m_intake;
    private VoltageOut voltagecontrol = new VoltageOut(10);
    private VoltageOut negvoltagecontrol = new VoltageOut(-10);
    private VoltageOut voltagestop = new VoltageOut(0);

    // References
    // private final Intake m_intake;

    public Hopper(Intake intake) {
        flopperMotor = new TalonFX(HopperConstants.FlopperCanID);
        leftTower = new TalonFX(HopperConstants.LeftTowerCANID);
        rightTower = new TalonFX(HopperConstants.RightTowerCANID);


        TalonFXConfiguration floppeConfiguration = new TalonFXConfiguration();
        floppeConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        floppeConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        flopperMotor.getConfigurator().apply(floppeConfiguration);

        TalonFXConfiguration toweConfiguration = new TalonFXConfiguration();
        toweConfiguration.CurrentLimits.SupplyCurrentLimit = 70;
        toweConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;

        leftTower.getConfigurator().apply(toweConfiguration);
        rightTower.getConfigurator().apply(toweConfiguration);

        canRange1 = new CANrange(HopperConstants.CanRangeID1);
        canRange2 = new CANrange(HopperConstants.CanRangeID2);

        m_intake = intake;
    }

    /** Sets both hopper motors to the same speed. */
    public void setSpeed(double flopperspeed, double towerspeed) {
        flopperMotor.set(-flopperspeed);
        leftTower.setControl(voltagecontrol);
        rightTower.setControl(negvoltagecontrol);
    }

    public void stop() {
        m_intake.setSpeed(0);
        //setSpeed(0.0, 0.0);

        flopperMotor.set(0);
        leftTower.setControl(voltagestop);
        rightTower.setControl(voltagestop); 
    }

    public void feed(double flopper, double tower) {
        m_intake.setSpeed(0.4);
        setSpeed(Math.abs(flopper), Math.abs(tower));
    }

    public void reverse(double flopper, double tower) {
        setSpeed(-Math.abs(flopper), -Math.abs(tower));
    }

    public Command runHopper(double flopper, double tower) {
        return run(() -> setSpeed(flopper, tower));
    }

    /** Runs both hoppers at default feed speed; stops when a ball is present. */
    public Command runHopperCommand() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                feed(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command feedCommand(double hopper, double tower) {
        return run(() -> feed(hopper, tower));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    /**
     * Checks if a ball is detected by the CANrange sensors.
     * 
     * @return true if ball is present.
     */
    public boolean hasBall() {
        // LaserMinDistance is 200.0 (mm), CANrange gets distance in meters
        boolean range1HasBall = canRange1.getDistance()
                .getValueAsDouble() < (HopperConstants.LaserMinDistance / 1000.0);
        boolean range2HasBall = canRange2.getDistance()
                .getValueAsDouble() < (HopperConstants.LaserMinDistance / 1000.0);
        return range1HasBall || range2HasBall;
    }

    /**
     * Runs Hopper to index balls. Stops if a ball is at the top (hasBall() is
     * true).
     */
    public Command runIndexCommand() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                feed(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command reverseHopper() {
        return run(() -> {
            if (hasBall()) {
                stop();
            } else {
                feed(-HopperConstants.HopperFeedSpeed, -HopperConstants.TowerFeedSpeed);
            }
        }).finallyDo(interrupted -> stop());
    }

    public Command runShootCommand() {
        return run(() -> {
            feed(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);
        }).finallyDo(interrupted -> stop());
    }

    /**
     * Runs Hopper to feed the shooter.
     * Ends when the hopper has been empty (no ball detected) for a set duration.
     */
    public Command runShootFeedCommand() {
        // Feed until balls stop passing through the hopper sensors.
        // We require at least one positive sensor detection (`hasBall()`) so we don't
        // immediately finish if the sensors start empty.
        final double kNoBallTimeoutSeconds = 1.0;

        final edu.wpi.first.wpilibj.Timer noBallTimer = new edu.wpi.first.wpilibj.Timer();
        noBallTimer.start();

        final boolean[] hasSeenBall = new boolean[] { false };

        return run(() -> {
            feed(HopperConstants.HopperFeedSpeed, HopperConstants.TowerFeedSpeed);

            if (hasBall()) {
                hasSeenBall[0] = true;
                // Reset the "no ball" timer whenever a ball is detected.
                noBallTimer.reset();
            }
        }).until(() -> hasSeenBall[0] && noBallTimer.hasElapsed(kNoBallTimeoutSeconds))
                .finallyDo(interrupted -> stop());
    }
}
