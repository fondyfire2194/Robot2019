package frc.robot.commands.Motion;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class ContinuousOutDrive extends Command {
	private double mySpeed;
	private double myTimeout;

	public ContinuousOutDrive(double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		mySpeed = speed;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(myTimeout);
		Robot.driveTrain.leftDriveOut(mySpeed);
		Robot.driveTrain.rightDriveOut(mySpeed);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return isTimedOut();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
