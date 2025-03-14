// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }

  public static class AlgaeConstants {
    public static final int ioSparkPort = 40;
    public static final int algaeArmSparkPort = 41;
    public static final int algaeArmEncoderChannelA = 0;
    public static final int algaeArmEncoderChannelB = 1;

    public static final double intakeSpeed = 0.5;
    public static final double outtakeSpeed = -0.5;
    public static final double algaeArmSpeed = 0.2;

    public static final int currentLimit = 35;

    public static final double upperLimitRotation = 0.5;
    public static final double lowerLimitRotation = 0;
    public static final double softStopDistance = 0.2;
  }
}
