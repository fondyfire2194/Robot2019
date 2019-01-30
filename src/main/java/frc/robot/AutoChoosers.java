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
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.commands.Auto.*;


public class AutoChoosers {

	public static SendableChooser<Integer> startPositionChooser;
	public static SendableChooser<Integer> secondHatchChooser;

	public static SendableChooser<Integer> testTrajectoryChooser;
	public static SendableChooser<Integer> trajectoryDirectionChooser;

	public static SendableChooser<Double> timeDelayChooser;

	public AutoChoosers() {
		testTrajectoryChooser = new SendableChooser<Integer>();

		testTrajectoryChooser.setDefaultOption("L " + TrajDict.leftStartNames[0], 0);
		testTrajectoryChooser.addOption("L " + TrajDict.leftStartNames[1], 1);
		testTrajectoryChooser.addOption("LC " + TrajDict.leftCenterStartNames[0],2);
		testTrajectoryChooser.addOption("RC " + TrajDict.rightCenterStartNames[0], 3);
		testTrajectoryChooser.addOption("R " + TrajDict.rightStartNames[0], 4);
		testTrajectoryChooser.addOption("R " + TrajDict.rightStartNames[1],5);
		testTrajectoryChooser.addOption(TrajDict.secondHatchNames[0],6);
		testTrajectoryChooser.addOption(TrajDict.secondHatchNames[1],7);
		testTrajectoryChooser.addOption(TrajDict.secondHatchNames[2],8);
		testTrajectoryChooser.addOption(TrajDict.secondHatchNames[3],9);

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		trajectoryDirectionChooser = new SendableChooser<Integer>();
		trajectoryDirectionChooser.setDefaultOption("FaceFieldMoveField", 1);
		trajectoryDirectionChooser.addOption("FaceWallMoveField", 2);
		trajectoryDirectionChooser.addOption("FaceFieldMoveWall", 3);
		trajectoryDirectionChooser.addOption("FaceWallMoveWall", 4);
		// SmartDashboard.putData("Trajectory Direction Chooser", trajectoryDirectionChooser);

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

	}
}
