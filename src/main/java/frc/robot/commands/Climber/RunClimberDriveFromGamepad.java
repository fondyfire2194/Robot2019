package frc.robot.commands.Climber;

import frc.robot.Robot;

import frc.robot.subsystems.ClimberDrive;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClimberDriveFromGamepad extends Command {

	public RunClimberDriveFromGamepad() {
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

		double yValue;
		double temp;
		if (Math.abs(Robot.m_oi.gamepad.getY()) > .1)
			yValue = Robot.m_oi.gamepad.getY();
		else
			yValue = 0;

		// square joystick and preserve sign
		temp = yValue * yValue;
		if (yValue < 0)
			temp = -temp;
		Robot.climberDrive.runClimberDrive(-yValue);// y up gives a negative value

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.climberLeg.runClimberLeg(0);


	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
