package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {
    public final SwerveSubsystem drivetrain;
    private final DoubleSupplier forwardX, forwardY, rotation, slider;
    private final SlewRateLimiter xLimiter, yLimiter, rLimiter;

    public SwerveJoystickCmd(
        SwerveSubsystem swerveSubsystem,
        DoubleSupplier forwardX,
        DoubleSupplier forwardY,
        DoubleSupplier rotation,
        DoubleSupplier slider
        ) {
        this.drivetrain = swerveSubsystem;
        this.forwardX = forwardX;
        this.forwardY = forwardY;
        this.rotation = rotation;
        this.slider = slider;

        xLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        yLimiter = new SlewRateLimiter(DrivetrainConstants.maxAcceleration);
        rLimiter = new SlewRateLimiter(DrivetrainConstants.maxAngularAcceleration);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize(){}

    @Override
    public void execute(){
        double xSpeed = forwardX.getAsDouble();
        double ySpeed = forwardY.getAsDouble();
        double rot = rotation.getAsDouble();
        double sliderVal = slider.getAsDouble();
        
        xSpeed = applyDeadbandAndLimiter(xSpeed, OperatorConstants.xDeadband, xLimiter, DrivetrainConstants.maxVelocity);
        ySpeed = applyDeadbandAndLimiter(ySpeed, OperatorConstants.yDeadband, yLimiter, DrivetrainConstants.maxVelocity);
        rot = applyDeadbandAndLimiter(rot, OperatorConstants.rDeadband, rLimiter, DrivetrainConstants.maxAngularVelocity);

        sliderVal = Math.min(sliderVal, 0.8);
        xSpeed *= (1 - sliderVal);
        ySpeed *= (1 - sliderVal);
        rot *= (1 - sliderVal);

        // xSpeed = MathUtil.applyDeadband(xSpeed, 0.1, 1);
        // ySpeed = MathUtil.applyDeadband(ySpeed, 0.1, 1);
        // rot = MathUtil.applyDeadband(rot, 0.3, 1);

        drivetrain.drive(
            xSpeed,
            ySpeed,
            rot,
            SwerveSubsystem.fieldRelativeStatus
        );
    }

    public double applyDeadbandAndLimiter(
        double value,
        double deadband,
        SlewRateLimiter limiter,
        double maxSpeed
    ) {
        value = Math.abs(value) < deadband ? 0 : value;
        return limiter.calculate(value)*maxSpeed;
    }

    @Override
    public void end(boolean interrupted){}

    @Override
    public boolean isFinished(){
        return false;
    }
}