package frc.robot.commands.AirCompressor;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.AirCompressor.*;
import frc.robot.subsystems.AirCompressor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StartCompressor extends Command {

	public StartCompressor() {
		// Use requires() here to declare subsystem dependencies
		requires(Robot.airCompressor);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		
		Robot.airCompressor.start();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
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