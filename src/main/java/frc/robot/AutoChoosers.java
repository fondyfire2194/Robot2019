/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoChoosers {

	public static SendableChooser<Integer> startPositionChooser;
	public static SendableChooser<Integer> secondHatchChooser;

	public static SendableChooser<Integer> testTrajectoryChooser;
	public static SendableChooser<Integer> trajectoryDirectionChooser;

	public static SendableChooser<Double> timeDelayChooser;

	public static SendableChooser<Integer> debugChooser;

	public AutoChoosers() {
		testTrajectoryChooser = new SendableChooser<Integer>();

		testTrajectoryChooser.setDefaultOption("L " + TrajDict.outsideStartNames[0], 0);
		testTrajectoryChooser.addOption("R " + TrajDict.outsideStartNames[0], 1);

		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchPickupNames[0], 2);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchPickupNames[0], 3);
		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchPickupNames[1], 4);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchPickupNames[1], 5);

		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchDeliveryNames[0], 6);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchDeliveryNames[0], 7);
		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchDeliveryNames[1], 8);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchDeliveryNames[1], 9);
		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchDeliveryNames[2], 10);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchDeliveryNames[2], 11);
		testTrajectoryChooser.addOption("L " + TrajDict.secondHatchDeliveryNames[3], 12);
		testTrajectoryChooser.addOption("R " + TrajDict.secondHatchDeliveryNames[3], 13);

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		trajectoryDirectionChooser = new SendableChooser<Integer>();
		trajectoryDirectionChooser.setDefaultOption("FaceFieldMoveField", 1);
		trajectoryDirectionChooser.addOption("FaceWallMoveField", 2);
		trajectoryDirectionChooser.addOption("FaceFieldMoveWall", 3);
		trajectoryDirectionChooser.addOption("FaceWallMoveWall", 4);
		// SmartDashboard.putData("Trajectory Direction Chooser",
		// trajectoryDirectionChooser);

		startPositionChooser = new SendableChooser<Integer>();
		startPositionChooser.setDefaultOption("DriverControl", 0);
		startPositionChooser.addOption("Left to LCS2", 1);
		startPositionChooser.addOption("LCenter", 2);
		startPositionChooser.addOption("RCenter", 3);
		startPositionChooser.addOption("Right to RCS2", 4);

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		secondHatchChooser = new SendableChooser<Integer>();

		secondHatchChooser.setDefaultOption("No Second Hatch", 0);
		secondHatchChooser.addOption("Cargo Ship 1", 1);
		secondHatchChooser.addOption("Cargo Ship 2", 2);
		secondHatchChooser.addOption("Cargo Ship 3", 3);
		secondHatchChooser.addOption("End Cargo Ship Far", 4);
		secondHatchChooser.addOption("Rocket", 5);

		SmartDashboard.putData("Second Hatch", secondHatchChooser);

		timeDelayChooser = new SendableChooser<Double>();

		timeDelayChooser.setDefaultOption("No Delay", 0.);
		timeDelayChooser.addOption("One Second", 1.);
		timeDelayChooser.addOption("Two Seconds", 2.);
		timeDelayChooser.addOption("Three Seconds", 3.);
		timeDelayChooser.addOption("Four Seconds", 4.);
		timeDelayChooser.addOption("Five Seconds", 5.);
		timeDelayChooser.addOption("Six Seconds", 6.);
		SmartDashboard.putData("Delay Chooser", timeDelayChooser);

		debugChooser = new SendableChooser<Integer>();

		debugChooser.setDefaultOption("None", 0);
		debugChooser.addOption("Robot", 1);
		debugChooser.addOption("DriveTrain", 2);
		debugChooser.addOption("Elevator", 3);
		debugChooser.addOption("GamePieceHandler", 4);
		debugChooser.addOption("PowerPanel", 5);
		debugChooser.addOption("RobotRotate", 6);
		debugChooser.addOption("VisionData", 7);

		SmartDashboard.putData("Debug", debugChooser);

	}
}
