/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.Pref;
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
  private boolean visionTargetSeen;
  private double targetBoxWidth;
  private double remainingDistanceFt;

  public JoystickArcadeDriveVision() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /**
   * Drive the robot using the camera to center it on the target. The leading side
   * front edge will hit first, causing the amps on that side to rise and the
   * gyro. This can be used to shut off the leading edge motor and the robot
   * should rotate until it is at the target angle.
   * 
   * 
   */
  // Called just before this Command runs the first time. Capture gyro yaw.
  // Use it to complete drive to target once a target was seen the lost de to
  // being too close to vcamera.
  @Override
  protected void initialize() {
    leftOverCurrentCount = 0;
    rightOverCurrentCount = 0;
    leftStalled = false;
    rightStalled = false;
    targetWasSeen = false;
    inVisionRange = false;
    Robot.limelightCamera.setLEDMode(LedMode.kforceOn);
    Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();
    remainingDistanceFt = -1;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    /**
     * 
     * get values from joystick turn value can also be taken from camera image or
     * gyro based on conditions
     */
    visionTargetSeen = Robot.limelightCamera.getIsTargetFound();
    targetBoxWidth = Robot.limelightCamera.getBoundingBoxWidth();

    throttleValue = Robot.m_oi.driverController.getY();
    if (Math.abs(throttleValue) < .15) {
      throttleValue = 0;
    }
    double temp = 0;
    temp = throttleValue * throttleValue;
    if (throttleValue < 0)
      throttleValue = temp;
    else
      throttleValue = -temp;

    if (targetWasSeen) {
      turnValue = Robot.m_oi.driverController.getTwist();
      temp = turnValue * turnValue;
      if (turnValue < 0)
        turnValue = -temp;
      else
        turnValue = temp;

      turnValue = turnValue * Robot.driveTrain.getDriverSlider();
    }

      /**
       * remember having seen target for switch to gyro when too close for camera to
       * have image switch to gyro when inside a max area value. Reset if joystick Y
       * is zero In vision range is image available but not greater area than preset
       * constant Too close for camera is used to switch to gyro if image was
       * previously seen
       * 
       */

      if (visionTargetSeen)
        targetWasSeen = true;

      inVisionRange =visionTargetSeen;//Robot.visionData.inGoodVisionDistanceRange();

      // tooCloseForCamera = visionTargetSeen && Robot.visionData.tooCloseToCamera();

      // in vision zone keep gyro target angle current to switch
      // over to gyro when too close for camera

      if (inVisionRange) {
        Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();

        if (Robot.limelightOnEnd) {
          turnValue = Robot.limelightCamera.getdegVerticalToTarget() * Pref.getPref("VisionKp");
        } else {
          turnValue = Robot.limelightCamera.getdegRotationToTarget() * Pref.getPref("VisionKp");
        }
      }

      if (targetWasSeen && !visionTargetSeen)
        turnValue = Robot.driveTrain.getCurrentComp();// gyro

       double leftValue = throttleValue + turnValue;
       double rightValue = throttleValue - turnValue;

      Robot.driveTrain.leftDriveOut(leftValue);
      Robot.driveTrain.rightDriveOut(rightValue);
      SmartDashboard.putBoolean("TWS", targetWasSeen);
      SmartDashboard.putBoolean("TIVR", inVisionRange);
      SmartDashboard.putBoolean("TTCFC", tooCloseForCamera);
      SmartDashboard.putNumber("TDSA", Robot.driveTrain.driveStraightAngle);
      SmartDashboard.putNumber("TVAL", turnValue);
    
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
    targetWasSeen = false;
    tooCloseForCamera = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
