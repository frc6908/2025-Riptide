// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MobilityAuton;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.FlipFieldRelativity;
import frc.robot.commands.FlipFieldRelativity2;
import frc.robot.commands.IntakeAlgae;
import frc.robot.commands.MoveArm;
import frc.robot.commands.OuttakeAlgae;
import frc.robot.commands.ResetArmEncoder;
import frc.robot.commands.ResetNavX;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.AlgaeMechanism;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SendableChooser<Command> autoChooser;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final SwerveSubsystem m_drivetrain = new SwerveSubsystem();
  private final AlgaeMechanism m_algaeMech = new AlgaeMechanism(AlgaeConstants.ioSparkPort, AlgaeConstants.algaeArmSparkPort);

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {


     autoChooser = AutoBuilder.buildAutoChooser();
     NamedCommands.registerCommand("AlgaeIntake", new IntakeAlgae(m_algaeMech));
     NamedCommands.registerCommand("AlgaeOuttake", new OuttakeAlgae(m_algaeMech));
     NamedCommands.registerCommand("ArmDown", new MoveArm(m_algaeMech, false));
     NamedCommands.registerCommand("ArmUp", new MoveArm(m_algaeMech,true));


    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");
    m_drivetrain.setDefaultCommand(new SwerveJoystickCmd(
      m_drivetrain, 
      () -> m_driverController.getLeftY(), 
      () -> m_driverController.getLeftX(), 
      () -> m_driverController.getRightX(),
      () -> m_driverController.getLeftTriggerAxis()
    ));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));
    
    // flip field relativity
    // m_driverController.x().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.x().whileTrue(new FlipFieldRelativity(m_drivetrain));
    m_driverController.a().whileTrue(new FlipFieldRelativity2(m_drivetrain));

    // reset navX heading
    m_driverController.y().whileTrue(new ResetNavX(m_drivetrain));

    // io algae
    m_operatorController.b().whileTrue(new IntakeAlgae(m_algaeMech));
    m_operatorController.x().whileTrue(new OuttakeAlgae(m_algaeMech));

    // arm movemenet
    m_operatorController.a().whileTrue(new MoveArm(m_algaeMech, false));
    m_operatorController.y().whileTrue(new MoveArm(m_algaeMech, true));
    m_operatorController.rightBumper().whileTrue(new ResetArmEncoder(m_algaeMech));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    //return MobilityAuton.exampleAuto(m_drivetrain);
    return autoChooser.getSelected();
  }
}
