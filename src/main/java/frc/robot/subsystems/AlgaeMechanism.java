package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeMechanism extends SubsystemBase {
    private final SparkMax ioSpark;
    private final SparkMax algaeArmSpark;
    private final Encoder algaeArmEncoder;

    public AlgaeMechanism(
        int ioSparkPort,
        int algaeArmSparkPort
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        algaeArmSpark = new SparkMax(algaeArmSparkPort, MotorType.kBrushless);
        algaeArmEncoder = new Encoder(AlgaeConstants.algaeArmEncoderChannelA, AlgaeConstants.algaeArmEncoderChannelB);
        
        configureMotor(ioSpark, IdleMode.kBrake, AlgaeConstants.currentLimit);
        configureMotor(algaeArmSpark, IdleMode.kBrake, AlgaeConstants.currentLimit);
    }

    public void configureMotor(
        SparkMax spark,
        IdleMode idleMode,
        int currentLimit
        ) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
            .idleMode(idleMode)
            .smartCurrentLimit(currentLimit);
        spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    public void setIOSpark(double speed) {
        ioSpark.set(speed);
    }

    public void stopIOSpark() {
        ioSpark.stopMotor();
    }

    public void setAlgaeArmSpark(double speed) {
        algaeArmSpark.set(speed);
    }

    public void stopAlgaeArmSpark() {
        algaeArmSpark.stopMotor();
    }

    public double getArmEncoderValue() {
        return algaeArmEncoder.get();
    }

    public void resetArmEncoder() {
        algaeArmEncoder.reset();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        
    }
}
