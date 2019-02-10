/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class TrajectoryPLS extends Command {

  int myStart;
  int myEnd;
  int trajectoryLength;
  int scan;
  public TrajectoryPLS(int startPct, int endPct) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
   myStart = startPct;
   myEnd = endPct;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    myStart = (myStart *Robot.activeLeftTrajectory.length()) /100;
     myEnd = (myEnd *Robot.activeLeftTrajectory.length())/100;
     scan =0;

     SmartDashboard.putNumber("Ten",myEnd);
     setTimeout(6.);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
scan++;
    SmartDashboard.putNumber("TSt",trajectoryLength++);
    Robot.trajectoryPulse  = Robot.currentTrajectorySegment >= myStart && Robot.currentTrajectorySegment <= myEnd;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return scan >20 && (!Robot.trajectoryRunning||Robot.currentTrajectorySegment > myEnd )||isTimedOut();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
