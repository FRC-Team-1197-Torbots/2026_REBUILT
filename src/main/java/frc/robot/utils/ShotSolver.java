package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.Optional;

/**
 * 2D kinematic Shot Solver for the FRC 2026 REBUILT game.
 * Uses realistic game piece ballistics and physics to figure out the ideal hood angle 
 * dynamically based on distance from the target Hub.
 */
public class ShotSolver {
    
    // --- Field & Robot Constants ---
    public static final double GRAVITY = 9.81; // m/s^2
    
    // The AprilTag plate is ~49.5 inches (1.25m) up. We'll set the target opening slightly higher.
    public static final double TARGET_HEIGHT = 1.35; // Approximate center of the Hub opening (meters)
    
    // You should tune this to the physical pivot point height of your Hood/Shooter on your 2026 REBUILT robot
    public static final double SHOOTER_HEIGHT = 0.5; // (meters)
    
    // --- Realistic Exit Velocity ---
    // A 4-inch diameter wheel spinning at 60 RPS (3600 RPM) has a surface speed of ~19.1 m/s.
    // Factor in standard foam ball slip/compression drop-off, you get ~15-18 m/s exit velocity.
    // Other teams generally find 16.0 m/s to be a highly realistic tuning point.
    public static final double DEFAULT_EXIT_VELOCITY = 16.0; // m/s
    
    public static class ShotSolution {
        public double angle;          // The required hood pitch angle in degrees
        public double velocity;       // The constant exit velocity used (m/s)
        public double timeOfFlight;   // Time until game piece impacts target (seconds)
        
        public ShotSolution(double angle, double velocity, double timeOfFlight) {
            this.angle = angle;
            this.velocity = velocity;
            this.timeOfFlight = timeOfFlight;
        }
    }
    
    /**
     * Solves for the required shooter angle to hit the hub.
     * 
     * @param distanceToTarget Flat ground distance to the target Hub (meters)
     * @param exitVelocity The exit velocity of the game piece (meters/second)
     * @return Optional containing the solution if a valid shot is physically possible at this range
     */
    public static Optional<ShotSolution> calculateShot(double distanceToTarget, double exitVelocity) {
        double x = distanceToTarget;
        double y = TARGET_HEIGHT - SHOOTER_HEIGHT; // vertical delta that the ball must travel
        double v = exitVelocity;
        
        // Using the kinematic equation of projectile trajectory:
        // y = x * tan(theta) - (g * x^2) / (2 * v^2 * cos^2(theta))
        //
        // Let u = tan(theta). We know sec^2(theta) = 1 / cos^2(theta) = 1 + u^2
        // So: y = x * u - (g * x^2 / (2 * v^2)) * (1 + u^2)
        // Let K = g * x^2 / (2 * v^2)
        // K * u^2 - x * u + (y + K) = 0
        
        double K = (GRAVITY * x * x) / (2 * v * v);
        
        // Quadratic equation coefficients for A*u^2 + B*u + C = 0
        double A = K;
        double B = -x;
        double C = y + K;
        
        double discriminant = B * B - 4 * A * C;
        
        if (discriminant < 0) {
            // Target is completely out of range for the current exit velocity (can't reach it physically)
            return Optional.empty();
        }
        
        // There are usually two mathematical roots for this problem (u = tan(theta)):
        double u1 = (-B + Math.sqrt(discriminant)) / (2 * A); // The higher "lob" angle
        double u2 = (-B - Math.sqrt(discriminant)) / (2 * A); // The lower "line-drive" angle
        
        double angleRad1 = Math.atan(u1);
        double angleRad2 = Math.atan(u2);
        
        // For FRC, we almost exclusively want the lowest possible line-drive angle (angleRad2)
        // for maximum speed and least environmental wind interference.
        double chosenAngle = angleRad2;
        
        // Failsafe: if the lower angle is somehow below 0 (shooting under the ground) 
        // fallback to the lob angle
        if (chosenAngle < 0 && angleRad1 >= 0) {
            chosenAngle = angleRad1;
        }
        
        double timeOfFlight = x / (v * Math.cos(chosenAngle));
        
        return Optional.of(new ShotSolution(
            Math.toDegrees(chosenAngle), 
            v, 
            timeOfFlight
        ));
    }
    
    /**
     * Overload using the default realistic exit velocity.
     * @param distanceToTarget Flat ground distance to the target Hub (meters)
     * @return The solved angle
     */
    public static Optional<ShotSolution> calculateShot(double distanceToTarget) {
        return calculateShot(distanceToTarget, DEFAULT_EXIT_VELOCITY);
    }
    
    /**
     * Convenience function to compute the required shot directly from Pose telemetry.
     */
    public static Optional<ShotSolution> getShotFromPose(Pose2d robotPose, Translation2d hubLocation) {
        double distance = robotPose.getTranslation().getDistance(hubLocation);
        return calculateShot(distance);
    }
}
