// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public final class MobilityAuton {
  /** Example static factory for an autonomous command. */
  public static Command exampleAuto(SwerveSubsystem m_drivetrain) {
    return Commands.sequence(
      new ResetNavX(m_drivetrain).withTimeout(1),
      new SwerveJoystickCmd(m_drivetrain,  () -> -0.3, () -> 0, () -> 0, () -> 0).withTimeout(10)
    );
  }

  private MobilityAuton() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}

