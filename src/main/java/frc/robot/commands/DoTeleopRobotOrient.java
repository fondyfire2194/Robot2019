package frc.robot.commands;

import frc.robot.Robot;


import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class DoTeleopRobotOrient extends InstantCommand {

	public DoTeleopRobotOrient() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.doTeleopOrient = true;
	
	}

}
