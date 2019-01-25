package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;

	public BuildTrajectory() {

	}

	public static Trajectory buildLeftFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;
		String tempPath = null;
		String filePath = "/home/lvuser/deploy/paths/";

		String usbFilePath = "/U/Traj19CSV/";

		// String leftExtension = ".left.pf1.csv";

		String leftExtension = "_left.csv";

		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		Robot.chosenFileName = "NONE";
		Robot.chosenFileName = tempPath + name + leftExtension;
		myLeftFile = new File(tempPath + name + leftExtension);

		if (myLeftFile.exists()) {
			Robot.bufferTrajName = "Loading";

			buffer = Pathfinder.readFromCSV(myLeftFile);

			Robot.chosenFileName = tempPath + name + leftExtension;
			Robot.buildOK = true;
			Robot.bufferTrajName = name;

		}
		Robot.buildInProgress = false;
		return buffer;

	}

	public static Trajectory buildRightFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;
		String tempPath = null;
		String filePath = "/home/lvuser/deploy/paths/";

		String usbFilePath = "/U/Traj19CSV/";

		// String rightExtension = ".right.pf1.csv";

		String rightExtension = "_right.csv";

		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		myRightFile = new File(tempPath + name + rightExtension);
		if (myRightFile.exists()) {
			buffer = Pathfinder.readFromCSV(myRightFile);

			Robot.chosenFileName = tempPath + name + rightExtension;
			Robot.buildOK = true;
			Robot.bufferTrajName = name;

		}
		Robot.buildInProgress = false;
		return buffer;
	}
}
