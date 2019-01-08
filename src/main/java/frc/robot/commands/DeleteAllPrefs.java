/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.Enumeration;
import java.util.Vector;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DeleteAllPrefs extends Command {
  Vector<String> v;
  Enumeration<String> e;

  public DeleteAllPrefs() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    v = new Vector<String>();// creating vector
    v = Robot.prefs.getKeys();
    e = v.elements();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    while (e.hasMoreElements()) {

      // System.out.println(e.nextElement());
      Robot.prefs.remove(e.nextElement());

    }
    // SmartDashboard.putNumber("Prefs Found", v.capacity());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
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
