package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeMechanism;

public class OuttakeAlgae extends Command {
    private final AlgaeMechanism m_algaeMech;

    public OuttakeAlgae(AlgaeMechanism algaeMech) {
        m_algaeMech = algaeMech;
        addRequirements(algaeMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_algaeMech.setIOSpark(AlgaeConstants.outtakeSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_algaeMech.stopIOSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
