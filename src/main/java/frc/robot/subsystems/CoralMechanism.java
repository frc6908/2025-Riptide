package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralMechanism extends SubsystemBase{
    private final SparkMax ioSpark;

    public CoralMechanism(
        int ioSparkPort
    ) {
        ioSpark = new SparkMax(ioSparkPort, MotorType.kBrushless);
        
        configureMotor(ioSpark, IdleMode.kBrake, CoralConstants.currentLimit);
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

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        
    }
}
