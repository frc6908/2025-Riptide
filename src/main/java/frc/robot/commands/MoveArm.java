package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.subsystems.AlgaeMechanism;

public class MoveArm extends Command {
    private final AlgaeMechanism m_algaeMech;
    private final boolean movingUp;
    public MoveArm(AlgaeMechanism algaeMech, boolean movingUp) {
        m_algaeMech = algaeMech;
        this.movingUp = movingUp;
        addRequirements(m_algaeMech);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double distanceToEdge;
        double speed = AlgaeConstants.algaeArmSpeed;
        if (movingUp) {
            // distanceToEdge = AlgaeConstants.upperLimitRotation - m_algaeMech.getArmEncoderValue();
            // speed = Math.min(1, distanceToEdge/AlgaeConstants.softStopDistance) * -AlgaeConstants.algaeArmSpeed;
        }
        else {
            // distanceToEdge = m_algaeMech.getArmEncoderValue() - AlgaeConstants.lowerLimitRotation;
            // speed = Math.min(1, distanceToEdge/AlgaeConstants.softStopDistance) * AlgaeConstants.algaeArmSpeed;
        }
        m_algaeMech.setAlgaeArmSpark(speed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_algaeMech.stopAlgaeArmSpark();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
