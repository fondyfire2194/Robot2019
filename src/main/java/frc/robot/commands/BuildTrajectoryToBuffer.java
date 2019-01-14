package frc.robot.commands;

import frc.robot.BuildTrajectory;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;

/**
 *
 */
public class BuildTrajectoryToBuffer extends InstantCommand {
	String myFileName;
	boolean myUsb;
	int myNumber;

	public BuildTrajectoryToBuffer(boolean usb, String filename) {
		super();
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myFileName = filename;
		myUsb = usb;
		
	}

	// Called once when the command executes
	protected void initialize() {
		Robot.bufferTrajectory = BuildTrajectory.buildFileName(myUsb, myFileName);

	}

}
