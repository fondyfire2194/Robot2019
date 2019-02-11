package frc.robot.commands.Teleop;

import frc.robot.Robot;


import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DoTeleopRotateToVision extends InstantCommand {

	public DoTeleopRotateToVision() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.doTeleopVisionOrient = true;
	
	}

}
