/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Pref;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickArcadeDriveVision extends Command {
  private int leftOverCurrentCount;
  private int rightOverCurrentCount;
  private int overCurrentCountMax = 10;
  private boolean leftStalled;
  private boolean rightStalled;
  private boolean targetWasSeen;
  private boolean tooCloseForCamera;
  private boolean inVisionRange;
  private double turnValue;
  private double throttleValue;

  public JoystickArcadeDriveVision() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /**
   * Drive the robot using the camera to center it on the target. The leading side
   * front edge will hit first, causing the amps on that side to rise and the gyro.
   *  This can be used to shut off the leading edge
   * motor and the robot should rotate until it is at the target angle.
   * 
   * 
   */
  // Called just before this Command runs the first time
  // If close enough to a valid target angle, use it to lock in the angle.
  @Override
  protected void initialize() {
    leftOverCurrentCount = 0;
    rightOverCurrentCount = 0;
    leftStalled = false;
    rightStalled = false;
    targetWasSeen = false;
    inVisionRange = false;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
/**
 * 
 * get values from joystick
 * turn value can also be taken from camera image or gyro based on conditions
 */
    throttleValue = -Robot.m_oi.driverController.getY();
    if (Math.abs(throttleValue) < .15) {
      throttleValue = 0;
    }

    if (!Robot.limelightCamera.getIsTargetFound()){
      turnValue = Robot.m_oi.driverController.getTwist() * Pref.getPref("JSTwistKp");
    }
    
/** remember having seen target for switch to gyro when too close for camera to have image
 * switch to gyro when inside a max area value. Reset if joystick Y is zero
 * In vision range is image available but not greater area than preset constant
 * Too close for camera is used to switch to gyro if image was previously seen
 * 
*/ 

if (Robot.limelightCamera.getIsTargetFound())
      targetWasSeen = true;

    inVisionRange = Robot.limelightCamera.getIsTargetFound()
        && Robot.limelightCamera.getTargetArea() < Constants.MAX_TARGET_AREA;

    if (!Robot.limelightCamera.getIsTargetFound())
      turnValue = Robot.m_oi.driverController.getTwist() * Pref.getPref("JSTwistKp");

    tooCloseForCamera = targetWasSeen && Robot.limelightCamera.getTargetArea() > Constants.MAX_TARGET_AREA;


    // in vision zone keep gyro target angle current to switch
    // over to gyro when too close for camera

    if (inVisionRange) {
      Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();

      if (Robot.limelightOnEnd) {
        turnValue = -Robot.limelightCamera.getdegVerticalToTarget() * Pref.getPref("VisionKp");
      } else {
        turnValue = -Robot.limelightCamera.getdegRotationToTarget() * Pref.getPref("VisionKp");
      }
    }
    if (tooCloseForCamera || targetWasSeen && !Robot.limelightCamera.getIsTargetFound())
      turnValue = Robot.driveTrain.getCurrentComp();//gyro
/** 
 * turning when robot bumpers hit target and amps rise
*/
    if (Robot.driveTrain.getLeftSideStalled()) {
      leftOverCurrentCount++;
    } else {
      leftOverCurrentCount = 0;
    }
    if (Robot.driveTrain.getRightSideStalled()) {
      rightOverCurrentCount++;
    } else {
      rightOverCurrentCount = 0;
    }
    if (leftOverCurrentCount > overCurrentCountMax) {
      leftStalled = true;
    }
    if (rightOverCurrentCount > overCurrentCountMax) {
      rightStalled = true;
    }
/**
 * Output to drive motors, Turn brakes off so robot coasts in the final stageS
 * 
 */
    double leftValue = throttleValue + turnValue;
    if (leftStalled) {
      leftValue = 0;
      Robot.driveTrain.setLeftSideDriveBrakeOn(false);
    }
    double rightValue = throttleValue - turnValue;
    if (rightStalled) {
      rightValue = 0;
      Robot.driveTrain.setRightSideDriveBrakeOn(false);
    }


    Robot.driveTrain.leftDriveOut(leftValue);
    Robot.driveTrain.rightDriveOut(rightValue);
    // SmartDashboard.putBoolean("TWS", targetWasSeen);
    // SmartDashboard.putBoolean("TIVR",inVisionRange);
    // SmartDashboard.putBoolean("TTCFC", tooCloseForCamera);
    // SmartDashboard.putNumber("TDSA",Robot.driveTrain.driveStraightAngle);
    // SmartDashboard.putNumber("TVAl", turnValue);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.driveTrain.arcadeDrive(0, 0);
    Robot.driveTrain.setLeftSideDriveBrakeOn(true);
    Robot.driveTrain.setRightSideDriveBrakeOn(true);
    targetWasSeen=false;
    tooCloseForCamera = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
