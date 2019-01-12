package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;

	// public BuildTrajectory() {

	// }

	public static Trajectory[] buildFileName(boolean usb, String name) {
		Trajectory active[] = new Trajectory[2];
		Robot.buildOK = false;
		String tempPath = null;
		String filePath = "/home/lvuser/Traj19CSV/";

		String usbFilePath = "/U/Traj19CSV/";

		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		Robot.chosenFileName = "NONE";

		myLeftFile = new File(tempPath + name + ".left.pf1.csv");
		myRightFile = new File(tempPath + name + ".right.pf1.csv");

		if (myLeftFile.exists() && myRightFile.exists()) {
			// Pathfinder.readFromCSV(myLeftFile);
			// Pathfinder.readFromCSV(myRightFile);
			 active[0] = Pathfinder.readFromCSV(myLeftFile);
			 active[1] = Pathfinder.readFromCSV(myRightFile);
			Robot.chosenFileName = tempPath + name + ".left.pf1.csv";
			Robot.buildOK = true;
		}
		return active;

	}
}
