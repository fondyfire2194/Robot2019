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

	public static SendableChooser<String> testTrajectoryChooser;
	public static SendableChooser<Integer> trajectoryDirectionChooser;

	public static SendableChooser<Double> timeDelayChooser;

	public AutoChoosers() {
		testTrajectoryChooser = new SendableChooser<String>();

		testTrajectoryChooser.setDefaultOption("Straight Line", "SimpleStraight");
		testTrajectoryChooser.addOption("Right Turn", "SimpleRightTurn");
		testTrajectoryChooser.addOption("Left Turn", "SimpleLeftTurn");
		testTrajectoryChooser.addOption("Field to Wall Test", "FieldToWallTest");

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		trajectoryDirectionChooser = new SendableChooser<Integer>();
		trajectoryDirectionChooser.setDefaultOption("FaceFieldMoveField", 1);
		trajectoryDirectionChooser.addOption("FaceWallMoveField", 2);
		trajectoryDirectionChooser.addOption("FaceFieldMoveWall", 3);
		trajectoryDirectionChooser.addOption("FaceWallMoveWall", 4);
		SmartDashboard.putData("Trajectory Direction Chooser", trajectoryDirectionChooser);

		startPositionChooser = new SendableChooser<Integer>();

		startPositionChooser.setDefaultOption("Left to LCS2", 0);
		startPositionChooser.addOption("LCenter", 1);
		startPositionChooser.addOption("RCenter", 2);
		startPositionChooser.addOption("Right to RCS2", 3);
		startPositionChooser.addOption("DriverControl", 4);

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		secondHatchChooser = new SendableChooser<Integer>();

		secondHatchChooser.setDefaultOption("No Second Hatch", 0);
		secondHatchChooser.addOption("Cargo Ship 1", 1);
		secondHatchChooser.addOption("Cargo Ship 2", 2);
		secondHatchChooser.addOption("Cargo Ship 3", 3);
		secondHatchChooser.addOption("End Cargo Ship Left", 4;
		secondHatchChooser.addOption("End Cargo Ship Right", 5);

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
