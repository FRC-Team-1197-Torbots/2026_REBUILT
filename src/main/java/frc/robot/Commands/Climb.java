package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;

public class Climb extends Command {
    
    public enum LEVELS { L0, L1, L2, L3}
    public LEVELS m_level = LEVELS.L0;
    private LEVELS targetLevel;

    public Climb(LEVELS level) {
        targetLevel = level;
    }

    @Override
    public void execute() {

    }

    @Override 
    public boolean isFinished() {
        return false;
    }
}
