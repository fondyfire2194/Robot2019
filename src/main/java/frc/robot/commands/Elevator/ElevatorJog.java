package frc.robot.commands.Elevator;

import frc.robot.Robot;
import frc.robot.Constants;


import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ElevatorJog extends Command {
	private double heightTarget;
	private double maxHeightIncrement = 6;

	public ElevatorJog() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double yValue;

		if (Math.abs(Robot.m_oi.gamepad.getY()) > .1)
			yValue = -Robot.m_oi.gamepad.getY();
		else
			yValue = 0;

		if (Robot.elevator.holdPositionInches < Constants.ELEVATOR_MAX_HEIGHT) {
			heightTarget = Robot.elevator.holdPositionInches + maxHeightIncrement * yValue;
			if (heightTarget > Constants.ELEVATOR_MAX_HEIGHT)
				Robot.elevator.holdPositionInches = Constants.ELEVATOR_MAX_HEIGHT;
			if (heightTarget < Constants.ALL_LOWER_HATCH_INCHES)
				Robot.elevator.holdPositionInches = Constants.ALL_LOWER_HATCH_INCHES;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {

		return Math.abs(Robot.m_oi.gamepad.getY()) < .1;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
