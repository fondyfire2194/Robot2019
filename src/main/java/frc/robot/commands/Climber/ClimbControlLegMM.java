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
 * When climbing, the climb angle will be controlled using the navX pitch feedback. Arm and leg amps will also be watched.
 * This command wll be run while the driver is holding a button.
 * 
 * When the driver releases the button, arm and leg go into motion magic mode to hold position. 
 * The arm target position is its current postion. The leg projects where it should be based on the arm angle.
 *  The leg needs to rise .322 inches for every degree the arm moves.
 * The arm start angle is latched and used to determine how far the arm has traveled.
 * The leg target position is set at leg starting position plus angle change * arm leg ratio.
 * 
 * 
 */

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Constants;

public class ClimbControlLegMM extends Command {
  private double mySpeed;
  private double myPitchAngle;
  private double lastPitchAngle;
  private double startArmAngle;
  private double startLegHeight;
  private double legIncrement = Constants.CLIMBER_ARM_LEG_RATIO;
  private double legTargetInches;

  public ClimbControlLegMM(double armSpeed, double pitchAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberLeg);

    mySpeed = armSpeed;
    myPitchAngle = pitchAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    startArmAngle = Robot.climberArm.getArmDegrees();
    legTargetInches = Robot.climberLeg.getLegInches();
    Robot.climberLeg.motionMagicRate = Constants.CLIMBER_LEG_CLIMB_RATE;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /**
     * Keep the leg in motion magic mode and project a new target every time the arm
     * angle changes by one degree. New target increment is the arm leg ratio
     * 
     * 
     */
    double robotLengthLegToArm = 35;
    double maxError = 5;
    double currentPitchAngle = Robot.driveTrain.getFilteredGyroPitch();
    double pitchError = myPitchAngle - currentPitchAngle;
    double pitchErrorRadians = Math.toRadians(pitchError);
    
    double legAngleComp = robotLengthLegToArm * Math.sin(pitchErrorRadians);

    if ((int) currentPitchAngle > (int) lastPitchAngle) {
      legTargetInches += legIncrement;

      Robot.climberLeg.legTargetInches = legTargetInches;

      if (pitchError < -maxError)
        Robot.climberLeg.legTargetInches = Robot.climberLeg.getLegInches();
    }
    lastPitchAngle = currentPitchAngle;
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.climberLeg.motionMagicRate = Constants.CLIMBER_LEG_POSITION_RATE;
    Robot.climberLeg.legTargetInches = Robot.climberLeg.getLegInches();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
