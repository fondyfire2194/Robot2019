/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/**
 * This command wll run once both axes are in their begin climb positions.
 * Prior to this, the arm will have two moves. One to a prepare angle and the other
 *  to a start of climb angle. The leg will have one move to a start of climb angle.
 * 
 * 
 * When climbing, the climb angle will be controlled using the navX pitch feedback. Arma nd leg amps will also be watched.
 * This command wll be run while the driver is holding a button.
 * 
 * 
 * 
 * 
 * 
 */

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Pref;

public class ClimbControlArm extends Command {
  private double myArmSpeed;
  private double myPitchAngle;

  public ClimbControlArm(double armSpeed, double pitchAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberArm);
    myArmSpeed = armSpeed;
    myPitchAngle = pitchAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

    Robot.climberArm.climberArmOut(myArmSpeed, true);

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /**
     * Pitch angle rises as front of robot rises If angle is too high, slow down arm
     * and speed up leg Error = Command - angle will give a negative result if the
     * front is too high So add to arm to slow it down
     * 
     * 
     */

    double pitchError = myPitchAngle - Robot.driveTrain.getFilteredGyroPitch();
    Robot.climberArm.climberArmOut(myArmSpeed - pitchError * Pref.getPref("ClimbAngleKp"), true);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberArm.armTargetDegrees = Robot.climberArm.getArmDegrees();
    Robot.climberArm.lastHoldDegrees = Robot.climberArm.getArmDegrees() + .01;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
