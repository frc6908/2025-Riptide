package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralMechanism;

public class IntakeCoral extends Command {
    private final CoralMechanism m_coralMech;
    
    public IntakeCoral(CoralMechanism coralMech) {
        m_coralMech = coralMech;
        addRequirements(coralMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_coralMech.setIOSpark(CoralConstants.intakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_coralMech.stopIOSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
