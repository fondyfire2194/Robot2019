package frc.robot.commands.Climber;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunClimberLegFromGamepad extends Command {
	private boolean myVel;

	public RunClimberLegFromGamepad(boolean vel) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.climberLeg);
		myVel = vel;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		if (myVel) {
			Robot.climberLeg.climberLeg.selectProfileSlot(1, 0);
		}
		Robot.climberLeg.climberLeg.configReverseSoftLimitEnable(false);
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
		yValue = temp / 2;

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.climberLeg.climberLegOut(0, false);
		Robot.climberLeg.climberLeg.configReverseSoftLimitEnable(true);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
