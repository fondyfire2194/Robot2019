/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import frc.robot.Robot;
import frc.robot.Pref;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

public class ClimberLegMotionMagic extends Command {
  public ClimberLegMotionMagic() {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    requires(Robot.climberLeg);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.climberLeg.climberLeg.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		Robot.climberLeg.climberLeg.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		Robot.climberLeg.climberLeg.selectProfileSlot(0, 0);

		Robot.climberLeg.climberLeg.selectProfileSlot(0, 0);

    Robot.climberLeg.legTargetInches = Robot.climberLeg.getLegInches();
		Robot.climberLeg.lastHoldInches = Robot.climberLeg.legTargetInches + .01;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (Robot.climberLeg.legTargetInches != Robot.climberLeg.lastHoldInches) {
			Robot.climberLeg.legMagicMotion(Robot.climberLeg.legTargetInches, Constants.CLIMBER_LEG_RATE);
			Robot.climberLeg.lastHoldInches = Robot.climberLeg.legTargetInches;

		}
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
