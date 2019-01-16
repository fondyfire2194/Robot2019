package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;

	// public BuildTrajectory() {

	// }

	public static Trajectory[] buildFileName( boolean usb, String name) {
		Trajectory buffer[] = new Trajectory[2];
		Robot.buildOK = false;
		String tempPath = null;
		String filePath = "/home/lvuser/deploy/";

		String usbFilePath = "/U/Traj19CSV/";

		String leftExtension ="left.pf1.csv";
		String rightExtension ="right.pf1.csv";
		
		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		Robot.chosenFileName = "NONE";

		myLeftFile = new File(tempPath + name + leftExtension);
		myRightFile = new File(tempPath + name + rightExtension);

		if (myLeftFile.exists() && myRightFile.exists()) {
			// Pathfinder.readFromCSV(myLeftFile);
			// Pathfinder.readFromCSV(myRightFile);
			 buffer[0] = Pathfinder.readFromCSV(myLeftFile);
			 buffer[1] = Pathfinder.readFromCSV(myRightFile);
			Robot.chosenFileName = tempPath + name + leftExtension;
			Robot.buildOK = true;
		}
		return buffer;

	}
}
