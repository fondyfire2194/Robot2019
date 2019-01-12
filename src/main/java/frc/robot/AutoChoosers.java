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
	public static SendableChooser<Integer> sameSideChooser;
	public static SendableChooser<Integer> startPositionChooser;
	public static SendableChooser<Integer> oppositeSidePriorityChooser;
	public static SendableChooser<Integer> testTrajectoryChooser;
	public static SendableChooser<Double> timeDelayChooser;

    public static void init(){
		testTrajectoryChooser = new SendableChooser<Integer>();

		testTrajectoryChooser.setDefaultOption("Test", 0);
		testTrajectoryChooser.addOption("LSW_L", 1);
		testTrajectoryChooser.addOption("LSW_L1 REV", 11);
		testTrajectoryChooser.addOption("LSW_L2", 12);

		testTrajectoryChooser.addOption("LSW_C", 2);
		testTrajectoryChooser.addOption("LSW_C1 REV", 21);

		testTrajectoryChooser.addOption("LSW_R", 3);

		testTrajectoryChooser.addOption("RSW_R", 4);
		testTrajectoryChooser.addOption("RSW_R1 REV", 41);
		testTrajectoryChooser.addOption("RSW_R2", 42);

		testTrajectoryChooser.addOption("RSW_C", 5);
		testTrajectoryChooser.addOption("RSW_C1 REV", 51);

		testTrajectoryChooser.addOption("RSW_L", 6);

		testTrajectoryChooser.addOption("LSC_L", 7);
		testTrajectoryChooser.addOption("LSC_R", 8);

		testTrajectoryChooser.addOption("RSC_R", 9);
		testTrajectoryChooser.addOption("RSC_L", 10);

		SmartDashboard.putData("Trajectory Chooser", testTrajectoryChooser);

		startPositionChooser = new SendableChooser<Integer>();

		startPositionChooser.setDefaultOption("Left", 1);
		startPositionChooser.addOption("Center", 2);
		startPositionChooser.addOption("Right", 3);

		SmartDashboard.putData("Start Position Chooser", startPositionChooser);

		sameSideChooser = new SendableChooser<Integer>();
		sameSideChooser.setDefaultOption("Do Scale if Available", 1);
		sameSideChooser.addOption("Do Switch if Available", 2);
		sameSideChooser.addOption("Cross Line", 3);

		SmartDashboard.putData("Same Side Priority Chooser", sameSideChooser);

		oppositeSidePriorityChooser = new SendableChooser<Integer>();

		oppositeSidePriorityChooser.setDefaultOption("Do Opposite Scale", 1);
		oppositeSidePriorityChooser.addOption("Do Opposite Switch", 2);
		oppositeSidePriorityChooser.addOption("Do Not Use Opposite", 3);

		SmartDashboard.putData("Opposite Side Chooser", oppositeSidePriorityChooser);

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
