package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Rotation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class SwerveModule extends SubsystemBase {
    private final SparkMax driveMotor;
    private final SparkMax rotationMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder rotationEncoder;

    private final CANcoder canCoder;
    private final double canCoderOffsetRadians;
    
    // PID
    private final PIDController rotationPIDController;
    private final PIDController drivePIDController;

    /* CREATE SWERVE MODULE */
    public SwerveModule(
      int driveMotorID,
      int rotationMotorID,
      int canCoderID,
      double canCoderOffsetRadians,
      boolean isDriveInverted,
      double x
    ) {
      // motor controllers
      driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
      rotationMotor = new SparkMax(rotationMotorID, MotorType.kBrushless);

      // motor controller config
      configureMotor(driveMotor, 
                      isDriveInverted, 
                      IdleMode.kBrake,
                      DrivetrainConstants.drivePositionConversionFactor, 
                      DrivetrainConstants.driveVelocityConversionFactor
      );
      configureMotor(rotationMotor, 
                      isDriveInverted, 
                      IdleMode.kBrake,
                      DrivetrainConstants.rotationPositionConversionFactor, 
                      DrivetrainConstants.rotationVelocityConversionFactor
      );
      driveMotor.clearFaults();
      rotationMotor.clearFaults();

      // PIDs
      drivePIDController = new PIDController(DrivetrainConstants.kPDrive, DrivetrainConstants.kIDrive, DrivetrainConstants.kDDrive);

      // rotationPIDController = new PIDController(DrivetrainConstants.kPRotation, DrivetrainConstants.kIRotation, DrivetrainConstants.kDRotation);
      rotationPIDController = new PIDController(x, DrivetrainConstants.kIRotation, DrivetrainConstants.kDRotation);
      rotationPIDController.setTolerance(DrivetrainConstants.kToleranceRotation);
      rotationPIDController.enableContinuousInput(-Math.PI, Math.PI);

      // CANcoder + relative encoders
      canCoder = new CANcoder(canCoderID);
      this.canCoderOffsetRadians = canCoderOffsetRadians;
      configureCanCoder();
      driveEncoder = driveMotor.getEncoder();
      rotationEncoder = rotationMotor.getEncoder();
    }

    /*
     * Configuration for SparkMax motor controller
     */
    public void configureMotor(
      SparkMax motorController,
      boolean isInverted,
      IdleMode idleMode,
      double positionConversionFactor,
      double velocityConversionFactor
    ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
          .inverted(isInverted)
          .idleMode(idleMode);
        config.encoder
          .positionConversionFactor(positionConversionFactor)
          .velocityConversionFactor(velocityConversionFactor);
        motorController.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /*
     * Configure CANcoder to operate with necessary behavior
     * [0,1) wrap range, CCW+ direction
     */ 
    public void configureCanCoder(){
      CANcoderConfiguration config = new CANcoderConfiguration();

      // makes range 0-1 for calculating radians
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      // makes CCW positive
      config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      
      // apply config
      canCoder.getConfigurator().apply(config);
    }

    /*
     * Set the desired state of the swerve module
     */
    public void setState(SwerveModuleState state){
      if (Math.abs(state.speedMetersPerSecond) < 0.0001){
        stop();
        return;
      }

      // minimizes rotation magnitude
      // state = optimize(state, getState().angle);

      // set rotation motor with PID controller
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), state.angle.getRadians()));

      // set drive motor speed
      // driveMotor.set(drivePIDController.calculate(getDriveVelocity(), state.speedMetersPerSecond/DrivetrainConstants.maxVelocity));
      driveMotor.set(drivePIDController.calculate(state.speedMetersPerSecond));
    }

    /*
     * Optimization function to prevent motors from turning more than 90 degrees
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
      var delta = desiredState.angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90){
        return new SwerveModuleState(-desiredState.speedMetersPerSecond, desiredState.angle.rotateBy(Rotation2d.kPi));
      }
      else{
        return new SwerveModuleState(desiredState.speedMetersPerSecond, desiredState.angle);
      }
    }

    /*
     * Get the current state of the swerve module
     */
    public SwerveModuleState getState(){
      return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getCANCoderRad()));
    }

    /*
     * Set rotation encoder offset to that of the CANcoder
     */
    public void initRotationOffset() {
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /*
     * @return current position of the CANCoder in radians
     */
    public double getCANCoderRad() {
      double absolutePosition = canCoder.getAbsolutePosition().getValueAsDouble();
      double angle = (2 * Math.PI * absolutePosition) - canCoderOffsetRadians;
      return angle % (2 * Math.PI);
    }

    /*
     * Set the rotation motor to a specific angle
     */
    public void setRotationMotorAnglePID(double angleRad) {
      rotationMotor.set(rotationPIDController.calculate(getCANCoderRad(), angleRad));
    }

    /*
     * Reset the encoder positions
     * Drive encoder to 0
     * Rotation encoder to CANCoder position
     */
    public void resetEncoder() {
      driveEncoder.setPosition(0);
      rotationEncoder.setPosition(getCANCoderRad());
    }

    /*
     * @return current drive velocity
     */
    public double getDriveVelocity(){
      return driveEncoder.getVelocity();
    }

    /*
     * @return current rotation velocity
     */
    public double getRotationVelocity(){
      return rotationEncoder.getVelocity();
    }

    /*
     * Stop the swerve module
     */
    public void stop(){
      driveMotor.stopMotor();
      rotationMotor.stopMotor();
    }

    /*
     * @return current drive position
     */
    public double getDrivePosition() {
      return driveEncoder.getPosition();
    }
  
    /*
     * @return current rotation position
     */
    public double getRotationPosition() {
      // return rotationEncoder.getPosition();
      return getCANCoderRad();
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }

  //   /**
  //  * Example command factory method.
  //  *
  //  * @return a command
  //  */
  // public Command exampleMethodCommand() {
  //   // Inline construction of command goes here.
  //   // Subsystem::RunOnce implicitly requires `this` subsystem.
  //   return runOnce(
  //       () -> {
  //         /* one-time action goes here */
  //       });
  // }
  // 
  // /**
  //  * An example method querying a boolean state of the subsystem (for example, a digital sensor).
  //  *
  //  * @return value of some boolean subsystem state, such as a digital sensor.
  //  */
  // public boolean exampleCondition() {
  //   // Query some boolean state, such as a digital sensor.
  //   return false;
  // }

  // @Override
  // public void simulationPeriodic() {
  //   // This method will be called once per scheduler run during simulation
  // }
}
