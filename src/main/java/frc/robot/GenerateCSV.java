package frc.robot;

import java.io.File;
import java.util.Arrays;
import java.util.Collections;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class GenerateCSV {
	String usbFilePath = "/media/sda1/JacisTrajCSV/";

	public GenerateCSV() {

	}
	

	public void generateTrajectories() {

		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.02, 6, 12.0, 10.0);

		Waypoint[] points = new Waypoint[] {

				new Waypoint(1.64, 23.1, Pathfinder.d2r(0)),

				new Waypoint(7.00, 24.00, Pathfinder.d2r(190)),

				new Waypoint(13.0, 23.0, Pathfinder.d2r(140)),

				new Waypoint(14.0, 21.54, Pathfinder.d2r(90)),

		};

		Trajectory trajectory = Pathfinder.generate(points, config);
		Trajectory reverseTrajectory = Pathfinder.generate(points, config);
		Collections.reverse(Arrays.asList(reverseTrajectory));
		
		File myTrajFile = new File (usbFilePath+"Traj.csv");
		Pathfinder.writeToCSV(myTrajFile, trajectory);

		File myRevTrajFile = new File (usbFilePath+"RevTraj.csv");
		Pathfinder.writeToCSV(myRevTrajFile, reverseTrajectory);

		// Wheelbase Width = 2.17 ft

		TankModifier modifier = new TankModifier(trajectory).modify(2.17);

		// Do something with the new Trajectories...

		Trajectory left = modifier.getLeftTrajectory();

		Trajectory right = modifier.getRightTrajectory();

		File myLFile = new File(usbFilePath + "LSW_LL.csv");
		Pathfinder.writeToCSV(myLFile, left);

		File myRFile = new File(usbFilePath + "LSW_LR.csv");
		Pathfinder.writeToCSV(myRFile, right);
		
		File myLTFile = new File(usbFilePath + "LSW_LL.traj");
		Pathfinder.writeToFile(myLTFile, left);

		File myRTFile = new File(usbFilePath + "LSW_LR.traj");
		Pathfinder.writeToFile(myRTFile, right);
	

	}

	// traj file
	public void makeTrajFile(Trajectory trajectory) {
		File myFile = new File("myfile.traj");
		Pathfinder.writeToFile(myFile, trajectory);
	}

	public void readTrajectoryFile(Trajectory trajectory) {
		File myFile = new File("myfile.traj");
		trajectory = Pathfinder.readFromFile(myFile);
	}

	// CSV File:
	public void makeCSVFile(Trajectory trajectory) {
		File myFile = new File("myfile.csv");
		Pathfinder.writeToCSV(myFile, trajectory);
	}

	public void readCSVFile(Trajectory trajectory) {
		File myFile = new File("myfile.csv");
		trajectory = Pathfinder.readFromCSV(myFile);
	}
}
