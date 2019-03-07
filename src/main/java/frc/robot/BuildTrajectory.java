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

	static String usbFilePath = "/U/DeepSpace/MPG4/output/";

	static String leftExtension = ".left.pf1.csv";

	static String rightExtension = ".right.pf1.csv";

	static String leftUSBExtension = "_left.csv";

	static String rightUSBExtension = "_right.csv";

	public BuildTrajectory() {

	}

	public static Trajectory buildLeftFileName(boolean usb, String name) {
		Robot.buildInProgress = true;
		Trajectory buffer = null;
		Robot.buildOK = false;
		String activeLeftExtension;
		String activeRightExtension;
		if (usb) {
			tempPath = usbFilePath;
			activeLeftExtension = leftUSBExtension;
			activeRightExtension = rightUSBExtension;
		} else {
			tempPath = filePath;
			activeLeftExtension = leftExtension;
			activeRightExtension = rightExtension;
		}

		Robot.chosenFileName = "NONE";
		Robot.chosenFileName = tempPath + name + activeLeftExtension;
		SmartDashboard.putString("None", Robot.chosenFileName);
		// if (Constants.usePathWeaver) {
		if (!usb) {
			myLeftFile = new File(tempPath + name + activeRightExtension);
		} else {
			myLeftFile = new File(tempPath + name + activeLeftExtension);
		}

		if (myLeftFile.exists()) {
			Robot.bufferTrajName = "Loading";

			buffer = Pathfinder.readFromCSV(myLeftFile);

			SmartDashboard.putString("CFN", Robot.chosenFileName);
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

		String activeLeftExtension;
		String activeRightExtension;
		if (usb) {
			tempPath = usbFilePath;
			activeLeftExtension = leftUSBExtension;
			activeRightExtension = rightUSBExtension;
		} else {
			tempPath = filePath;
			activeLeftExtension = leftExtension;
			activeRightExtension = rightExtension;
		}

		// if (Constants.usePathWeaver) {
		if (!usb) {
			myRightFile = new File(tempPath + name + activeLeftExtension);
		} else {
			myRightFile = new File(tempPath + name + activeRightExtension);
		}

		if (myRightFile.exists()) {
			buffer = Pathfinder.readFromCSV(myRightFile);

			Robot.chosenFileName = tempPath + name + activeRightExtension;
			Robot.buildOK = true;
			Robot.bufferTrajName = name;

		}
		Robot.buildInProgress = false;
		return buffer;
	}
}
