package frc.robot.commands.Climber;

import frc.robot.Robot;

import frc.robot.subsystems.ClimberArm;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClimberArmFromGamepad extends Command {
	private boolean myVel;

	public RunClimberArmFromGamepad(boolean vel) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.climberArm);
		myVel = vel;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (myVel) {
			Robot.climberArm.climberArm.selectProfileSlot(1, 0);
		}
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
		Robot.climberArm.climberArmOut(yValue, myVel);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.climberArm.climberArmOut(0, myVel);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
