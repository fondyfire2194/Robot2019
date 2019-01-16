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

	public static SendableChooser<Command> startPositionChooser;
	public static SendableChooser<Command> secondHatchChooser;

	public static SendableChooser<String> testTrajectoryChooser;
	public static SendableChooser<Integer> trajectoryDirectionChooser;

	public static SendableChooser<Double> timeDelayChooser;

	 public AutoChoosers() {
		testTrajectoryChooser = new SendableChooser<String>();

		testTrajectoryChooser.setDefaultOption("Straight One", "StraightOne");
		testTrajectoryChooser.addOption("Curve One", "CurveOne");
		testTrajectoryChooser.addOption("Curve Two", "CurveTwo");

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		trajectoryDirectionChooser = new SendableChooser<Integer>();
		trajectoryDirectionChooser.setDefaultOption("FaceFieldMoveField", 0);
		trajectoryDirectionChooser.addOption("FaceFieldMoveWall", 1);
		trajectoryDirectionChooser.addOption("FaceWallMoveField", 2);
		trajectoryDirectionChooser.addOption("FaceWallMoveWall", 3);
		SmartDashboard.putData("Trajectory Direction Chooser", trajectoryDirectionChooser);



		startPositionChooser = new SendableChooser<Command>();

		startPositionChooser.setDefaultOption("Left to LCS2", new LHab1ToLCS2());
		startPositionChooser.addOption("LCenter", new CHab1ToLC());
		startPositionChooser.addOption("RCenter", new CHab2ToRC());
		startPositionChooser.addOption("Right to RCS2", new RHab1ToRCS2());
		startPositionChooser.addOption("DriverControl", new SetUpDriverControl());

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		secondHatchChooser = new SendableChooser<Command>();

		secondHatchChooser.setDefaultOption("No Second Hatch", new DoNothing());
		secondHatchChooser.addOption("LC1", new LC1ToLLD());
		secondHatchChooser.addOption("LC2", new LC2ToRLD());

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
