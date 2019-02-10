/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;


import frc.robot.commands.Motion.*;
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

    public static int setOutsideStart() {
        int number = 1;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = "1 - Reset Encoders and Gyro";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, false, Robot.invertY);
        Robot.autonomousCommandName[number] = "2 - Trajectory To CS2 Line";
        number++;
        Robot.autonomousCommand[number] = new SetGyroOffset(180);
        Robot.autonomousCommandName[number] = "3 - Set Gyro Offset";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = "4 - Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanel();
        Robot.autonomousCommandName[number] = "5 - Place Panel";
        Robot.secondHatchIndex=1;
        return number;
    }

    public static int setMiddleStart() {
        int number = 1;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = "1 - Reset Encoders and Gyro";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = "2 - Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanel();
        Robot.autonomousCommandName[number] = "3 - Place Panel";
        Robot.secondHatchIndex=0;
        return number;

    }

 

    public static int pickUpSecondHatch(int start, int indexStart) {
        int number = indexStart + 1;
         switch (start) {
        case 1:
        case 4:
        SmartDashboard.putNumber("WTH",Robot.secondHatchIndex);
        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchIndex);
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Set Trajectory To CS";
        number++;
       Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, false, Robot.invertY);
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Move to Load Station Trajectory";
        number++;
        Robot.autonomousCommand[number] = new RobotOrient(180, .5, true, 3);
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Orient to Load Station";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Move To Load Station";
        number++;
        Robot.autonomousCommand[number] = new PickUpHatchPanel();
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Pick Up Panel";
        break;
        case 2:
        case 3:
            Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, false, Robot.invertY);
            Robot.autonomousCommandName[number] = String.valueOf(number) +" - Move to Load Station Trajectory";
            number++;
            Robot.autonomousCommand[number] = new RobotOrient(180, .5, true, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) +" - Orient to Load Station";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Move To Load Station";
            number++;
            Robot.autonomousCommand[number] = new PickUpHatchPanel();
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Pick Up Panel";
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
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Set Trajectory To CS";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, false, Robot.invertY);
        Robot.autonomousCommandName[number] = String.valueOf(number) +" - Trajectory To CS";
        number++;

        switch (hatchSelected) {

        case 1:
        case 2:
            Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, .5, true, 2);
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Orient To CS";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Place Panel";
            break;
        case 3:
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) +" - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Place Panel";
            break;

        case 4:
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8., 5, false, 3);
            Robot.autonomousCommandName[number] =String.valueOf(number) + " - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanel();
            Robot.autonomousCommandName[number] = String.valueOf(number) +" - Place Panel";

        }
        return number;
    }

    public static void updateStatus(int number) {
        for (int i = 0; i <= number; i++) {
            SmartDashboard.putString("AutoCommand " + String.valueOf(i), Robot.autonomousCommandName[i]);

        }

    }
}
