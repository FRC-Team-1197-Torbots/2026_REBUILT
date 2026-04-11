package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * The AimingManager is responsible for taking the Robot's current drivetrain
 * pose
 * and the Field Zone to calculate exactly where the Turret (Yaw) and Hood
 * (Pitch)
 * should be aiming.
 * 
 * By pulling this out of Turret.java and Hood.java, those subsystems can remain
 * "dumb" motor controllers, while this class handles all the complex trig and
 * targeting.
 */
public class AimingManager extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final ZoneDetection zoneDetection;

    // References to the hardware subsystems
    private final Turret leftturret;
    private final Turret rightturret;
    private final Shooter leftShooter;
    private final Shooter rightShooter;
    private final Hood leftHood;
    private final Hood rightHood;

    // Shoot-on-the-Move Settings

    private LinearFilter filter = LinearFilter.singlePoleIIR(0.7, 0.02);

    public AimingManager(CommandSwerveDrivetrain drivetrain, ZoneDetection zoneDetection,
            Turret leftTurret, Turret righTurret, Shooter leftShooter, Shooter rightShooter,
            Hood leftHood, Hood rightHood) {
        this.drivetrain = drivetrain;
        this.zoneDetection = zoneDetection;
        this.leftturret = leftTurret;
        this.rightturret = righTurret;
        this.leftShooter = leftShooter;
        this.rightShooter = rightShooter;
        this.leftHood = leftHood;
        this.rightHood = rightHood;
    }

    @Override
    public void periodic() {

        if (zoneDetection == null || zoneDetection.getAlliance().isEmpty()) {
            return;
        }

        // // 1. Calculate LEFT Hood & Shooter
        calculateAndApplyAiming(leftturret, leftShooter, leftHood, "Left");

        // // 2. Calculate RIGHT Hood & Shooter
        calculateAndApplyAiming(rightturret, rightShooter, rightHood, "Right");
    }

    private void calculateAndApplyAiming(Turret turret, Shooter shooter, Hood hood, String sideName) {

        if (turret == null && shooter == null)
            return;

        double distanceMeters = turret.getDistanceToTarget();

        double calculatedRPS;
        double calculatedHoodTicks;

        // If we are passing the ball (in neutral zone or opponent's zone), use hardcoded high speeds
        // and fixed hood angles. Otherwise, dynamically calculate based on distance to speaker.
        if (zoneDetection != null && (zoneDetection.getZone() == ZoneDetection.ZONE.NEUTRAL || zoneDetection.isOpponentZone())) {
            calculatedRPS = 60.0;
            calculatedHoodTicks = 8.0;
        } else {
            calculatedRPS = calculateRps(distanceMeters);
            calculatedHoodTicks = calculateHoodTicks(distanceMeters);
        }

        if (shooter != null) {
            shooter.setShooterSpeed(calculatedRPS);
        }

        if (hood != null) {
            if (shooter != null && shooter.isShooting()) {
                hood.setTargetAngle(calculatedHoodTicks);
            } else {
                hood.setTargetAngle(0.0);
            }
        }

        // Telemetry
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/Distance_m",
        // distanceMeters);
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/RPS",
        // calculatedRPS);
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/Hood Ticks",
        // calculatedHoodTicks);
        // SmartDashboard.putNumber("AimingManager/" + sideName + "/Hood Ticks Actual",
        // hood.getEncoderTicks());
    }

    private double calculateHoodTicks(double d) {
        double a = 0.085;
        double b = 0.893;
        double c = -1.1785;

        // At extremely close ranges under 2 meters, keep hood safely retracted
        if (d < 2.0) {
            return 0.0;
        }
        
        return filter.calculate((a * d * d) + (b * d) + c);
    }

    private double calculateRps(double d) {
        // https://docs.google.com/spreadsheets/d/12vaU1FRqllZlERNKd85nal3VIQaEh6twuFeA2sOHeNw/edit?pli=1&gid=0#gid=0
        double a = 4.8252;
        double b = -2.0846;
        double c = 38.708;
        // return a * d * d + b * d + c;
        return a * d + c;
    }
}


