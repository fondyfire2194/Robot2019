package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;
	static String tempPath = null;
	static String filePath = "/home/lvuser/deploy/paths/";

	static String usbFilePath = "/U/Traj19CSV/";

	static String leftExtension = ".left.pf1.csv";

	// static String leftExtension = "_left.csv";
	static String rightExtension = ".right.pf1.csv";

	// String rightExtension = "_right.csv";

	public BuildTrajectory() {

	}

	public static Trajectory buildLeftFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;

		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		Robot.chosenFileName = "NONE";
		Robot.chosenFileName = tempPath + name + leftExtension;
		if (Constants.usePathWeaver) {
			myLeftFile = new File(tempPath + name + rightExtension);
		} else {
			myLeftFile = new File(tempPath + name + leftExtension);
		}
		if (myLeftFile.exists()) {
			Robot.bufferTrajName = "Loading";

			buffer = Pathfinder.readFromCSV(myLeftFile);

			SmartDashboard.putString("CFN", Robot.chosenFileName);
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

		if (usb)
			tempPath = usbFilePath;
		else
			tempPath = filePath;

		if (Constants.usePathWeaver) {
			myRightFile = new File(tempPath + name + leftExtension);
		} else {
			myRightFile = new File(tempPath + name + rightExtension);
		}

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
