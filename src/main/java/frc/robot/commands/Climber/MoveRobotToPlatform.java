/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MoveRobotToPlatform extends Command {
  public MoveRobotToPlatform() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double throttleValue = Robot.m_oi.driverController.getY();
    double temp = 0;

    if (Math.abs(throttleValue) < .15)
      throttleValue = 0;
      temp = throttleValue * throttleValue;
      if (throttleValue < 0)
        throttleValue = temp;
      else
        throttleValue = -temp;

        Robot.driveTrain.arcadeDrive(throttleValue/10, 0);
        Robot.climberDrive.climberDriveOut(throttleValue/10);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
