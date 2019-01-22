package frc.robot.commands.Elevator;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class ResetElevatorPosition extends InstantCommand {

	public ResetElevatorPosition() {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.elevator.resetElevatorPosition();
	}

}
