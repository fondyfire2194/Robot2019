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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class JoystickArcadeDrive extends Command {
  public JoystickArcadeDrive() {
    requires(Robot.driveTrain);
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double throttleValue = -Robot.m_oi.driverController.getY();
    double turnValue = Robot.m_oi.driverController.getTwist();
    SmartDashboard.putNumber("Lout", throttleValue);

    if (Math.abs(throttleValue) < .15)
      throttleValue = 0;
    if (Math.abs(turnValue) < .15)
      turnValue = 0;
    if (!Robot.autoRunning)
      // Robot.driveTrain.arcadeDrive(throttleValue, turnValue *
      // Pref.getPref("JSTwistKp"));
      Robot.driveTrain.arcadeDrive(throttleValue, turnValue * Robot.driveTrain.getDriverSlider());
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
