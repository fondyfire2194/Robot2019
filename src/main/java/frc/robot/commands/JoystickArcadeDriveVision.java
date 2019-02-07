/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import frc.robot.Pref;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickArcadeDriveVision extends Command {
  private int leftOverCurrentCount;
  private int rightOverCurrentCount;
  private int overCurrentCountMax = 10;
  private double gyroOffLimit = 5.;
  private boolean leftStalled;
  private boolean rightStalled;

  public JoystickArcadeDriveVision() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  /**
   * Drive the robot using the camera to center it on the target. The leading side
   * front edge will hit first, causing the amps on that side to rise and the gyro
   * is rotating bit to turn on. This can be used to shut off the leading edge
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
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {

    double turnValue = 0;
    double throttleValue = -Robot.m_oi.driverController.getY();
    double closestTargetAngle = Robot.visionData.atTargetAngle();
    boolean robotOnLine = closestTargetAngle != 999;

    if (!robotOnLine) {
      SmartDashboard.putBoolean("Locked",false);
      turnValue = Robot.m_oi.driverController.getTwist() * Pref.getPref("JSTwistKp");

      if (Math.abs(throttleValue) < .15) {
        throttleValue = 0;
      }
      if (Math.abs(turnValue) < .15) {
        turnValue = 0;
      }
    }
    else {
      SmartDashboard.putBoolean("Locked",true);
      turnValue = Robot.limelightCamera.getdegRotationToTarget() * Pref.getPref("VisionKp");
    }
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
    SmartDashboard.putBoolean("Locked",false);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
