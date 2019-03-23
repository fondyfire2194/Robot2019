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

public class ClimbControlLeg extends Command {
  private double mySpeed;
  private double myPitchAngle;
  private double lastPitchAngle;

  public ClimbControlLeg(double armSpeed, double pitchAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberLeg);

    mySpeed = armSpeed;
    myPitchAngle = pitchAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is s -cheduled to run
  @Override
  protected void execute() {
    /**
     * Pitch angle rises as front of robot rises If angle is too high, slow down arm
     * and speed up leg Error = Command - angle will give a negative result if the
     * front is too high So subtract from leg to speed it up
     * 
     * 
     */
    double maxError=5;
    double loopTime = .02;
    double currentPitchAngle = Robot.driveTrain.getFilteredGyroPitch();
    double pitchError = myPitchAngle - currentPitchAngle;
    double pitchChange = lastPitchAngle - currentPitchAngle;
    lastPitchAngle = currentPitchAngle;
    double currentSpeed = (mySpeed * Constants.CLIMBER_ARM_LEG_RATIO) / Constants.MAX_LEG_INCHES_PER_SEC;
    double pitchRateOfChange = pitchChange / loopTime;
    
    Robot.climberLeg.climberLegOut(
        currentSpeed + (pitchError * Robot.climberArm.getDriverSliderClimb() + pitchRateOfChange * .001), true);
  /**
   * if leg is getting ahead, pitch error will negative so leg must slow down
   * if pitch error is too negative, stop leg
   * 
   */
  
        if (pitchError < -maxError)
      Robot.climberLeg.climberLegOut(0, true);

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberLeg.legTargetInches = Robot.climberLeg.getLegInches();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
