package frc.robot;

import java.io.File;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * USB files come from Motion Profile Generator instead of PathWeaver So
 * selecting usb files will make the switch Constants.usePathWeaver goes away
 * 
 * 
 */
public class BuildTrajectory {
	public static File myLeftFile;
	public static File myRightFile;
	static String tempPath = null;

	static String filePath = "/home/lvuser/deploy/paths/";
	static String usbFilePath = "/U/DeepSpace/PathWeaver/output/";

	static String leftExtension = ".left.pf1.csv";
	static String rightExtension = ".right.pf1.csv";

	public BuildTrajectory() {

	}

	public static Trajectory buildLeftFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;
		Robot.chosenFileName = "NONE";

		if (!usb) {
			tempPath = filePath;
		} else {
			tempPath = usbFilePath;
		}

		if (Constants.usePathWeaver) {
			myLeftFile = new File(tempPath + name + rightExtension);
			Robot.chosenFileName = tempPath + name + rightExtension;
		} else {
			myLeftFile = new File(tempPath + name + leftExtension);
			Robot.chosenFileName = tempPath + name + leftExtension;
		}

		if (myLeftFile.exists()) {
			Robot.bufferTrajName = "Loading";
			buffer = Pathfinder.readFromCSV(myLeftFile);
			Robot.buildOK = true;
			Robot.bufferTrajName = name;
			SmartDashboard.putNumber("BuffLgth", buffer.length());

		}
		Robot.buildInProgress = false;
		return buffer;

	}

	public static Trajectory buildRightFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;
		if (!usb) {
			tempPath = filePath;
		} else {
			tempPath = usbFilePath;
		}

		if (Constants.usePathWeaver) {
			myRightFile = new File(tempPath + name + leftExtension);
			Robot.chosenFileName = tempPath + name + leftExtension;
		} else {
			myRightFile = new File(tempPath + name + rightExtension);
			Robot.chosenFileName = tempPath + name + rightExtension;
		}

		if (myRightFile.exists()) {
			buffer = Pathfinder.readFromCSV(myRightFile);
			Robot.buildOK = true;
			Robot.bufferTrajName = name;
		}
		Robot.buildInProgress = false;
		return buffer;
	}
}
