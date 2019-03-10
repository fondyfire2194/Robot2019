package frc.robot.commands.Climber;

import frc.robot.Robot;

import frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClimberArmFromGamepad extends Command {

	public void RunClimberArmFromGamepad() {
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

		double xValue;
		double temp;
		if (Math.abs(Robot.m_oi.gamepad_test.getX()) > .1)
			xValue = Robot.m_oi.gamepad.getX();
		else
			xValue = 0;

		// square joystick and preserve sign
		temp = xValue * xValue;
		if (xValue < 0)
			temp = -temp;
		Robot.climber.runClimberArm(xValue);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.climber.runClimberArm(0);


	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
