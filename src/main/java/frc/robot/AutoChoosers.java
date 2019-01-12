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
	public static SendableChooser<Command> testTrajectoryChooser;
	public static SendableChooser<Boolean> stopChooser;
	public static SendableChooser<Double> timeDelayChooser;

	public static void init() {
		testTrajectoryChooser = new SendableChooser<Command>();

		testTrajectoryChooser.setDefaultOption("Left To CS2",new LHab1ToLShipMid());
		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		startPositionChooser = new SendableChooser<Command>();

		startPositionChooser.setDefaultOption("Left",new LHab1ToLShipMid());
		startPositionChooser.addOption("LCenter", new LCenHabToLShipFront());
		startPositionChooser.addOption("RCenter", new RCenHabToRShipFront());
		startPositionChooser.addOption("Right", new RHab1ToRShipMid());

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		stopChooser = new SendableChooser<Boolean>();

		stopChooser.setDefaultOption("Two Hatch", true);
		stopChooser.addOption("One Hatch", false);

		SmartDashboard.putData("Number of Hatches", stopChooser);

		timeDelayChooser = new SendableChooser<Double>();

		timeDelayChooser.setDefaultOption("No Delay", 0.);
		timeDelayChooser.addOption("One Second", 1.);
		timeDelayChooser.addOption("Two Seconds", 2.);
		timeDelayChooser.addOption("Three Seconds", 3.);
		timeDelayChooser.addOption("Four Seconds", 4.);
		timeDelayChooser.addOption("Five Seconds", 5.);

		SmartDashboard.putData("Delay Chooser", timeDelayChooser);

	}
}
