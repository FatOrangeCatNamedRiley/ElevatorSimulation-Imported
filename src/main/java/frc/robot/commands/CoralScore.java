// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class CoralScore extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final CoralScore subsystem;
  private final double setPoint;
  private final boolean isLeft, isTop;
  /**
   * Creates a new ExampleCommand. 
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem s, Elevator e, double setElevatorPoint, boolean isItLeft, boolean isItTop) {
    subsystem = s;
    elevator = e;
    setPoint = setElevatorPoint;
    isLeft = isItLeft;
    isTop = isItTop;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the height point of the elevator
    // change light color eventualy
    // change x/y position depending on if it's left or right

    if (isLeft && isTop)           // Leve 3 left side
    {
      Commands.parallel(new MoveToSetpoint(3.0));
    } else if (!isLeft && isTop) { // Level 3 right side
      Commands.parallel(new MoveToSetpoint(3.0));
    } else if (isLeft && !isTop) { // Level 2 left side
      Commands.parallel(new MoveToSetpoint(2.0));
    } else {                       // Level 2 right side
      Commands.parallel(new MoveToSetpoint(2.0));
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
