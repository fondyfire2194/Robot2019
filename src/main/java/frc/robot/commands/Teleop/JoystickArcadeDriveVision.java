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
  private boolean targetWasSeen;
  private double turnValue;
  private double throttleValue;
  private boolean visionTargetSeen;
  private int targetSeenCtr;
  private double visionTargetError;
  

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
    targetWasSeen = false;
    targetSeenCtr = 0;
    Robot.limelightCamera.setLEDMode(LedMode.kforceOn);
    Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();
    Robot.visionCompJoystick = true;
    visionTargetError = 0;
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

    if (!targetWasSeen) {
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
     * have image
     * 
     */

    if (visionTargetSeen)
      targetSeenCtr++;
    else
      targetSeenCtr = 0;
    if (targetSeenCtr >= 10)
      targetWasSeen = true;

    // in vision zone keep gyro target angle current to switch
    // over to gyro when too close for camera or once the vision error is near 0

    if (visionTargetSeen) {
      Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();

      if (Robot.limelightOnEnd) {
        visionTargetError = Robot.limelightCamera.getdegVerticalToTarget();
      } else {
        visionTargetError = Robot.limelightCamera.getdegRotationToTarget();
      }
      turnValue = visionTargetError * Pref.getPref("VisionKp");
    }

    if ((targetWasSeen && !visionTargetSeen))
      turnValue = Robot.driveTrain.getCurrentComp();// gyro

    double leftValue = throttleValue + turnValue;
    double rightValue = throttleValue - turnValue;

    Robot.driveTrain.leftDriveOut(leftValue);
    Robot.driveTrain.rightDriveOut(rightValue);

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
    Robot.visionCompJoystick = false;
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
