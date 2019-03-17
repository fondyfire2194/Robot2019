/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.ClimberArm;
import frc.robot.Robot;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ClimberArmPosition extends Command {

private double myDegrees;
private double myRate;


  public ClimberArmPosition(double degrees, double rate) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myDegrees = degrees;
    myRate = rate;
    requires (Robot.climberArm);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
        /* Config the peak and nominal outputs, 12V means full */
        myRate = .2;
		Robot.climberArm.climberArm.configNominalOutputForward(0, 0);
		Robot.climberArm.climberArm.configNominalOutputReverse(0, 0);
		Robot.climberArm.climberArm.configPeakOutputForward(myRate, 0);
		Robot.climberArm.climberArm.configPeakOutputReverse(-myRate, 0);

		


    Robot.climberArm.climberArm.set(ControlMode.Position,myDegrees);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.climberArm.armInPosition();
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
