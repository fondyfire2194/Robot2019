/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.Pref;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Robot.motionType;

public class RobotPosition extends Command {

  private double mySpeed;
  private boolean myEndItNow;
  private double myTimeout;
  private double rampIncrement;
  private double startInPositionTime;
  private boolean inPositionTimed;

  private boolean doneAccelerating;
  public static double currentMaxSpeed;

  public double myDistance;
  private double drivePositionKp;
  public boolean decelerate;
  private double remainingFeet;
  private double useSpeed;
  private motionType myType;
  private double speedAtSlowdown;

  // distances are in feet
  // speeds are in per unit where .25 = 25%
  // inPositionband is in feet
  public RobotPosition(double distance, double speed, motionType type, boolean endItNow, double timeout) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.driveTrain);

    mySpeed = speed;
    myEndItNow = endItNow;
    myDistance = distance;
    myTimeout = timeout;
    myType = type;

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    rampIncrement = mySpeed / 50;
    setTimeout(myTimeout);
    Robot.isPositioning = true;
    drivePositionKp = Pref.getPref("DrivePositionKp");
    if (myType == motionType.incremental) {
      myDistance = myDistance + Robot.driveTrain.getLeftFeet() + Robot.driveTrain.getPositionError();
      // SmartDashboard.putNumber("INC",99);
    }
    Robot.positionTargetFt = myDistance;
    Robot.driveTrain.setDriveBrakeOn(true);
    currentMaxSpeed = 0;
    doneAccelerating = false;
    decelerate = false;
    inPositionTimed = false;
    startInPositionTime = 0;
    currentMaxSpeed = 0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    if (!doneAccelerating && !decelerate) {
      currentMaxSpeed = currentMaxSpeed + rampIncrement;
      if (currentMaxSpeed > mySpeed) {
        currentMaxSpeed = mySpeed;
        doneAccelerating = true;
      }
    }
    remainingFeet = myDistance - Robot.driveTrain.getLeftFeet();

    if (!decelerate && Math.abs(remainingFeet) * drivePositionKp < currentMaxSpeed){
      decelerate = true;
      speedAtSlowdown = currentMaxSpeed;
    }
    if (decelerate) {
      currentMaxSpeed = (speedAtSlowdown * Math.abs(remainingFeet) * drivePositionKp);
    }

    if (Robot.driveTrain.getPositionError() < 0)
      useSpeed = -currentMaxSpeed;
    else
      useSpeed = currentMaxSpeed;
    Robot.driveTrain.arcadeDrive(useSpeed, Robot.driveTrain.getCurrentComp());
    if (Robot.driveTrain.inPosition() && startInPositionTime == 0)
      startInPositionTime = Timer.getFPGATimestamp();

    inPositionTimed = startInPositionTime != 0 && Robot.driveTrain.inPosition()
        && Timer.getFPGATimestamp() - startInPositionTime > .25;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return isTimedOut() || myEndItNow || inPositionTimed;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.isPositioning = false;
    Robot.stopPositioning = false;
    Robot.driveTrain.arcadeDrive(0, 0);
    doneAccelerating = false;
    decelerate = false;
    currentMaxSpeed = 0;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
