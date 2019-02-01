/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import frc.robot.commands.Auto.*;
import frc.robot.commands.Auto.LCHabStart.*;
import frc.robot.commands.Auto.RCHabStart.*;
import frc.robot.commands.Auto.LHabStart.*;
import frc.robot.commands.Auto.RHabStart.*;
import frc.robot.commands.Motion.*;
import frc.robot.commands.Auto.SecondPanel.LoadToCS1;
import frc.robot.commands.Auto.SecondPanel.LoadToCS2;
import frc.robot.commands.Auto.SecondPanel.LoadToCS3;
import frc.robot.commands.Auto.SecondPanel.LoadToECSFar;
import frc.robot.commands.Auto.SecondPanel.LoadToECSNear;
import frc.robot.commands.HatchPanels.*;
import frc.robot.commands.Trajectories.BufferToActiveTrajectory;
import frc.robot.commands.Trajectories.PickAndRunTrajectory;;

/**
 * Add your docs here.
 */
public class AutoCommands {

    public AutoCommands() {

    }

    /**
     * Commands can be held and resumed at the robot command step level but not at
     * the Command Group level, so the steps should be broken down to the point
     * where they can be paused if the end point is not clear of other robots
     * 
     * 1 - move to face cargo ship 2 - move to ship and place panel 3 - move to face
     * load station 4 - pick up panel 5 - move to face cargo ship 6 - move to ship
     * and place panel
     * 
     */
    /**
     * The trajectory preload task can set up any global variables like a Robot
     * invertY boolean and rotate angle then use that in subsequent commands to
     * adapt the trajectories and any rotate angles
     * 
     * Then left and right starts look the same
     * 
     * 
     */
    // public static int setLeftStart() {

    // Robot.autonomousCommand[1] = new LHabToLCS2();
    // Robot.autonomousCommand[2] = new LCS2ToLLoad();
    // Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 2, 3);
    // Robot.autonomousCommand[4] = new MoveToLoadStation(4);
    // Robot.autonomousCommand[5] = new PickUpHatchPanel(5);
    // numberOfCommands = 5;
    // return numberOfCommands;
    // }
    // public static int setRightStart() {

    // Robot.autonomousCommand[1] = new RHabToRCS2();
    // Robot.autonomousCommand[2] = new RCS2ToRLoad();
    // Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 5, 3);
    // Robot.autonomousCommand[4] = new MoveToLoadStation(4);
    // Robot.autonomousCommand[5] = new PickUpHatchPanel(5);
    // numberOfCommands = 5;
    // return numberOfCommands;
    // }

    public static int setOutsideStart() {
        int number = 1;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = "Reset Encoders and Gyro";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, false, Robot.invertY);
        Robot.autonomousCommandName[number] = "Trajectory To CS2 Line";
        number++;
        Robot.autonomousCommand[number] = new SetGyroOffset(180);
        Robot.autonomousCommandName[number] = "Set Gyro Offset";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = "Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanel();
        Robot.autonomousCommandName[number] = "Place Panel";
        return number;
    }

    public static int setMiddleStart() {
        int number = 1;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = "Reset Encoders and Gyro";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = "Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanel();
        Robot.autonomousCommandName[number] = "Place Panel";
        return number;

    }

    // public static int setLeftCenterStart() {
    //     Robot.autonomousCommand[1] = new LCHabToLECS();
    //     Robot.autonomousCommand[2] = new LCToLLoad();
    //     Robot.autonomousCommand[3] = new RobotOrient(180, .5, true, 5);
    //     Robot.autonomousCommand[4] = new MoveToLoadStation(4);
    //     Robot.autonomousCommand[5] = new PickUpHatchPanel();
    //     int numberOfCommands = 5;
    //     return numberOfCommands;
    // }

    // public static int setRightCenterStart() {
    //     Robot.autonomousCommand[1] = new RCHabToRECS();
    //     Robot.autonomousCommand[2] = new RCToRLoad();
    //     Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 5, 3);
    //     Robot.autonomousCommand[4] = new MoveToLoadStation(4);
    //     Robot.autonomousCommand[5] = new PickUpHatchPanel();
    //     int numberOfCommands = 5;
    //     return numberOfCommands;
    // }

    public static int pickUpSecondHatch(int start, int indexStart) {
        int number = indexStart + 1;
        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchIndex);
        Robot.autonomousCommandName[number] = "Set Move to Load Station Trajectory";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, false, Robot.invertY);
        Robot.autonomousCommandName[number] = "Move to Load Station Trajectory";
        number++;
        switch (start) {
        case 1:
        case 4:
        Robot.autonomousCommand[number] = new RobotOrient(180, .5, true, 3);
        Robot.autonomousCommandName[number] = "Orient to Load Station";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = "Move To Load Station";
        number++;
        Robot.autonomousCommand[5] = new PickUpHatchPanel();
        Robot.autonomousCommandName[number] = "Pick Up Panel";
        break;
        case 2:
        case 3:
            Robot.autonomousCommand[number] = new RobotOrient(180, .5, true, 3);
            Robot.autonomousCommandName[number] = "Orient to Load Station";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] = "Move To Load Station";
            number++;
            Robot.autonomousCommand[5] = new PickUpHatchPanel();
            Robot.autonomousCommandName[number] = "Pick Up Panel";
            break;
        }
        return number;
    }

    // Hatch selected uses the int fron AutoChoosers, indexStart is the last command
    // from above

    public static int deliverSecondHatch(int hatchSelected, int indexStart) {
        int number = indexStart;
        number++;
        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchIndex + 1);
        Robot.autonomousCommandName[number] = "Set Trajectory To CS";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, false, Robot.invertY);
        Robot.autonomousCommandName[number] = "Trajectory To CS";
        number++;

        switch (hatchSelected) {

        case 1:
        case 2:
            Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, .5, true, 2);
            Robot.autonomousCommandName[number] = "Orient To CS";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] = "Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] = "Place Panel";
            break;
        case 3:
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] = "Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] = "Place Panel";
            break;

        case 4:
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] = "Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] = "Place Panel";

        }
        return number;
    }

    public static void updateStatus(int number) {
        for (int i = 0; i <= number; i++) {
            SmartDashboard.putString("AutoCommand " + String.valueOf(i), Robot.autonomousCommandName[i]);

        }

    }
}
