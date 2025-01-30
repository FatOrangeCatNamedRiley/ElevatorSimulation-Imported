// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.Elevator;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveToSetpoint;
import frc.robot.commands.RunIntake;
import frc.robot.commands.RunOuttake;
import frc.robot.commands.SetElevatorSetpoint;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final XboxController m_driverController = new XboxController(Constants.OI.kDriverControllerPort);

  private final CommandXboxController m_simulatorController = new CommandXboxController(0);

  private final JoystickSim simulatorJoystick = new JoystickSim(0);

  private Intake intake = new Intake();

  //private static Elevator m_elevator;


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //m_elevator = Robot.getElevator();
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {


    //Move to bottom
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
              .whileTrue(new MoveToSetpoint(0));
    m_simulatorController.button(1).whileTrue(new MoveToSetpoint(0));


    //Move to 1.5 (wherever that is)
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
              .whileTrue(new MoveToSetpoint(1.5));
    m_simulatorController.button(2).whileTrue(new MoveToSetpoint(1.5));


    //Move to 3.0 (wherever that also is)
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
                .whileTrue(new MoveToSetpoint(3));
    m_simulatorController.button(3).whileTrue(new MoveToSetpoint(3));

    //Run Intake
    new JoystickButton(m_driverController, XboxController.Button.kY.value)
                .whileTrue(new RunIntake(intake));
    m_simulatorController.button(4).whileTrue(new RunIntake(intake));

    //Run Outtake
    //new JoystickButton(m_driverController, XboxController.Button.kY.value)
    //            .whileTrue(new RunOuttake(intake));
    m_simulatorController.button(5).whileTrue(new RunOuttake(intake));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
