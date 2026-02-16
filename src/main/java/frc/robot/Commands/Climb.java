package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.Constants;

public class Climb extends Command {
    
    public enum LEVELS { L0, L1, L2, L3}
    private LEVELS targetLevel;
    private final Climber m_climber;
    
    // Tracking current logical level to enforce sequential climbing
    private static LEVELS currentSimulatedLevel = LEVELS.L0;

    public Climb(Climber climber, LEVELS level) {
        m_climber = climber;
        targetLevel = level;
        addRequirements(m_climber);
    }

    @Override
    public void initialize() {
        // Enforce sequential Climbing
        // Allow L0 (Get Off/Reset) from anywhere
        if (targetLevel == LEVELS.L0) {
            currentSimulatedLevel = LEVELS.L0;
        } 
        // Allow going up one level
        else if (targetLevel.ordinal() == currentSimulatedLevel.ordinal() + 1) {
            currentSimulatedLevel = targetLevel;
        }
        // Allow going down one level
        else if (targetLevel.ordinal() == currentSimulatedLevel.ordinal() - 1) {
            currentSimulatedLevel = targetLevel;
        }
        // Otherwise, ignore the command (safety)
        else {
            // Option: Print warning or just do nothing.
            // For now, we update target to current so it holds position
             targetLevel = currentSimulatedLevel;
        }
    }

    @Override
    public void execute() {
        double targetMeters = 0;
        switch (targetLevel) {
            case L0: targetMeters = Constants.ClimberConstants.L0; break;
            case L1: targetMeters = Constants.ClimberConstants.L1; break;
            case L2: targetMeters = Constants.ClimberConstants.L2; break;
            case L3: targetMeters = Constants.ClimberConstants.L3; break;
        }
        m_climber.moveToPosition(targetMeters);
    }

    @Override 
    public boolean isFinished() {
        double targetMeters = 0;
        switch (targetLevel) {
            case L0: targetMeters = Constants.ClimberConstants.L0; break;
            case L1: targetMeters = Constants.ClimberConstants.L1; break;
            case L2: targetMeters = Constants.ClimberConstants.L2; break;
            case L3: targetMeters = Constants.ClimberConstants.L3; break;
        }
        
        double currentError = Math.abs(m_climber.getHeight() - targetMeters);
        return currentError < Constants.ClimberConstants.Tolerance;
    }
}
