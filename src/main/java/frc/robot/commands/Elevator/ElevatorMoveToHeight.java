
package frc.robot.commands.Elevator;

import frc.robot.Robot;


import edu.wpi.first.wpilibj.command.Command;


/**
 *
 */
public class ElevatorMoveToHeight extends Command {
	private double myHeight;
	private boolean atPosition;
	private double atPositionBand = 3;


	public ElevatorMoveToHeight(double height) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myHeight = height;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.moveIsUp = myHeight > Robot.elevator.holdPositionInches;
		Robot.elevator.moveIsDown = myHeight < Robot.elevator.holdPositionInches;
		Robot.elevator.holdPositionInches = myHeight;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		atPosition = (!Robot.elevator.moveIsUp && !Robot.elevator.moveIsDown)
				|| Robot.elevator.moveIsUp
						&& Robot.elevator.getElevatorPositionInches() > myHeight - atPositionBand
				|| Robot.elevator.moveIsDown
						&& Robot.elevator.getElevatorPositionInches() < myHeight + atPositionBand;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return atPosition;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {

	}
}
