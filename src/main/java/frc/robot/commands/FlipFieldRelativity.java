package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.AlgaeConstants;
//import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.subsystems.SwerveSubsystem;

public class FlipFieldRelativity extends Command {
    private final SwerveSubsystem m_drivetrain;

    public FlipFieldRelativity(SwerveSubsystem drivetrain) {
        m_drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_drivetrain.setFieldRelativity(true);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {

    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
