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

public class ClimbControlArmAndLeg extends Command {
  private double mySpeed;
  private double myPitchAngle;
  private double legSpeed;
  private double armSpeed;

  public ClimbControlArmAndLeg(double legSpeed, double pitchAngle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    mySpeed = legSpeed;//in/sec
    myPitchAngle = pitchAngle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {

  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double pitchError = myPitchAngle - Robot.driveTrain.getFilteredGyroPitch();
/**
 * arm lift producd in inches decreases as arm moves down so arm needs to speed up as the lift proceeds. 
 * So another multiplier is needed ?which is a function of arm angle ?
 * 
 * 
 * 
 */


    double armLiftRatio = 2; //starting lift point deg/in
    double armKp = .3;
    double armStartLiftAngle = 22;
    double armFinishLiftAngle = 54;
    legSpeed = mySpeed;

    /** 
     * pitchError will be + if below target and - if above
     * 
     * so + meand speed up arm. Leg will run constant speed unless error gets too negative.
     * 
     * 
     * */


    armSpeed = (legSpeed * armLiftRatio) - armKp * pitchError;//in/sec * deg/in = deg/sec

    Robot.climber.climberArmOut(armSpeed, true);

    Robot.climber.climberLegOut(legSpeed, true);
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
