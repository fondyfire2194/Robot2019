/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
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
        Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(5,Constants.POSITION_RATE, false, 3);
        Robot.autonomousCommandName[number] = "5 - Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
        Robot.autonomousCommandName[number] = "6 - Place Panel";
        Robot.secondHatchPickupIndex = 0;
        Robot.secondHatchDeliverIndex = 1;
 
        return number;
    }

    public static int setMiddleStart() {
        int number = 1;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = "1 - Reset Encoders and Gyro";
        number++;
        Robot.autonomousCommand[number] = new RobotDriveToTarget(11.25,Constants.POSITION_RATE, false, 3);
        Robot.autonomousCommandName[number] = "2 - Drive To Cargo Ship";
        number++;
        Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
        Robot.autonomousCommandName[number] = "3 - Place Panel";
        
        Robot.secondHatchDeliverIndex = 0;
        return number;

    }

    public static int pickUpSecondHatch(int start, int indexStart) {
        int number = indexStart + 1;
        switch (start) {
        case 1:
        case 4:
            Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchPickupIndex);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Set Trajectory To LS";
            number++;
            Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, true, Robot.invertY);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move to Load Station Trajectory";
            number++;
            Robot.autonomousCommand[number] = new RobotOrient(180, Constants.ORIENT_RATE, true, 1.5);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station";
            number++;
            Robot.autonomousCommand[number] = new ResetEncodersAuto();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Encoders";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(6,Constants.POSITION_RATE, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Load Station";
            number++;
            Robot.autonomousCommand[number] = new PickUpHatchPanel();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Pick Up Panel";
            break;
        case 2:
        case 3:
            Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, true, Robot.invertY);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move to Load Station Trajectory";
            number++;
            Robot.autonomousCommand[number] = new RobotOrient(180, Constants.ORIENT_RATE, true, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station";
            number++;
            Robot.autonomousCommand[number] = new ResetEncodersAuto();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Encoders";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(6, Constants.POSITION_RATE, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Load Station";
            number++;
            Robot.autonomousCommand[number] = new PickUpHatchPanel();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Pick Up Panel";
            break;
        }
        return number;
    }

    // Hatch selected uses the int fron AutoChoosers, indexStart is the last command
    // from above

    public static int deliverSecondHatch(int hatchSelected, int indexStart) {
        int number = indexStart;
        number++;
        Robot.autonomousCommand[number] = new ResetEncodersAndGyro();
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Gyro and Encoders";
        number++;
        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchDeliverIndex);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Set Trajectory To CS";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, false, Robot.invertY);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Trajectory To CS";
        number++;
        Robot.autonomousCommand[number] = new SetGyroOffset(180);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Set Gyro Offset";
        number++;

        switch (hatchSelected) {

        case 1:
        case 2:
            Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS";
            number++;
            Robot.autonomousCommand[number] = new ResetEncodersAuto();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Encoders";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(2,Constants.POSITION_RATE, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place Panel";
            break;
        case 3:
            Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS";
            number++;
            Robot.autonomousCommand[number] = new ResetEncodersAuto();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Encoders";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(2, Constants.POSITION_RATE, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place Panel";
            break;

        case 4:
            Robot.autonomousCommand[number] = new RobotOrient(90 + Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS";
            number++;
            Robot.autonomousCommand[number] = new ResetEncodersAuto();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Reset Encoders";
            number++;
            Robot.autonomousCommand[number] = new RobotDriveToTarget(8.,Constants.POSITION_RATE, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To CS";
            number++;
            Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place Panel";

        }
        return number;
    }

    public static void updateStatus(int number) {
        for (int i = 0; i <= number; i++) {
            SmartDashboard.putString("AutoCommand " + String.valueOf(i), Robot.autonomousCommandName[i]);

        }

    }
}
