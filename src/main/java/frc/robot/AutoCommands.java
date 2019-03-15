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
import frc.robot.commands.Auto.DriveAndPickUpPanel;
import frc.robot.commands.Auto.DriveAndPlacePanelShip;
import frc.robot.commands.Trajectories.BufferToActiveTrajectory;
import frc.robot.commands.Trajectories.PickAndRunTrajectory;
import frc.robot.commands.Auto.DriveAndPlacePanelRocket;

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
        double firstPlaceDistance = 7.;
        int number = 1;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, false, Robot.invertY);
        Robot.autonomousCommandName[number] = "1 - Trajectory To CS2 Line";
        number++;
        Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
        Robot.autonomousCommandName[number] = "2 - Orient to Cargo Ship " + String.valueOf(Robot.sideAngle);
        number++;
        Robot.autonomousCommand[number] = new DriveAndPlacePanelShip(firstPlaceDistance, Constants.POSITION_RATE,
                Robot.sideAngle, false, 3);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                + String.valueOf(firstPlaceDistance);

        // Robot.autonomousCommand[number] = new RobotDriveToTarget(5,
        // Constants.POSITION_RATE, Robot.sideAngle, false, 3);
        // Robot.autonomousCommandName[number] = "3 - Drive To Cargo Ship";
        // number++;
        // Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
        // Robot.autonomousCommandName[number] = "4 - Place Panel";
        Robot.secondHatchPickupIndex = 0;
        Robot.secondHatchDeliverIndex = 1;

        return number;
    }

    public static int setMiddleStart() {
        double firstPlaceDistance = 11.;
        int number = 1;
        Robot.autonomousCommand[number] = new DriveAndPlacePanelShip(firstPlaceDistance, Constants.POSITION_RATE, 0,
                false, 3);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                + String.valueOf(firstPlaceDistance);

        // Robot.autonomousCommand[number] = new RobotDriveToTarget(11.,
        // Constants.POSITION_RATE, 0, false, 3);
        // Robot.autonomousCommandName[number] = "1 - Drive To Cargo Ship";
        // number++;
        // Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
        // Robot.autonomousCommandName[number] = "2 - Place Panel";
        Robot.secondHatchDeliverIndex = 0;
        return number;

    }

    public static int pickUpSecondHatch(int start, int indexStart) {
        int number = indexStart + 1;
        double pickupDistance = 7.;
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
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station 180";
            number++;
        //     Robot.autonomousCommand[number] = new DriveAndPickUpPanel(pickupDistance, Constants.POSITION_RATE, 180,
        //             Robot.gph.getLeftHatchDetected() || Robot.gph.getRightHatchDetected(), 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Load and Pickup "
                    + String.valueOf(pickupDistance);
            // Robot.autonomousCommand[number] = new RobotDriveToTarget(7,
            // Constants.POSITION_RATE, 180, false, 3);
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To
            // Load Station";
            // number++;
            // Robot.autonomousCommand[number] = new PickUpHatchPanel();
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Pick Up
            // Panel";
            break;
        case 2:
        case 3:
            pickupDistance = 7.;
            Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, true, Robot.invertY);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move to Load Trajectory";
            number++;
            Robot.autonomousCommand[number] = new RobotOrient(180, Constants.ORIENT_RATE, true, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station 180";
            number++;
            Robot.autonomousCommand[number] = new DriveAndPickUpPanel(pickupDistance, Constants.POSITION_RATE, 180,
                    false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Load Station and PickUp "
                    + String.valueOf(pickupDistance);

            // Robot.autonomousCommand[number] = new RobotDriveToTarget(6.,
            // Constants.POSITION_RATE, 180, false, 3);
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To
            // Load Station";
            // number++;
            // Robot.autonomousCommand[number] = new PickUpHatchPanel();
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Pick Up
            // Panel";
            // break;
        }
        return number;
    }

    // Hatch selected uses the int fron AutoChoosers, indexStart is the last command
    // from above

    public static int deliverSecondHatch(int hatchSelected, int indexStart) {
        int number = indexStart;
        number++;
        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchDeliverIndex);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Set Trajectory To CS";
        number++;
        Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, false, Robot.invertY);
        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Trajectory To CS";
        number++;
        switch (hatchSelected) {

        case 1:
        case 2:
            double secondPlaceDistance = 3.;
            Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS "
                    + String.valueOf(Robot.sideAngle);
            number++;
            Robot.autonomousCommand[number] = new DriveAndPlacePanelShip(secondPlaceDistance, Constants.POSITION_RATE,
                    Robot.sideAngle, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                    + String.valueOf(secondPlaceDistance);

            // Robot.autonomousCommand[number] = new RobotDriveToTarget(2,
            // Constants.POSITION_RATE, Robot.sideAngle, false,
            // 3);
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To
            // CS";
            // number++;
            // Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place
            // Panel";
            break;
        case 3:
            secondPlaceDistance = 3.;
            Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS "
                    + String.valueOf(Robot.sideAngle);
            ;
            number++;
            Robot.autonomousCommand[number] = new DriveAndPlacePanelShip(secondPlaceDistance, Constants.POSITION_RATE,
                    Robot.sideAngle, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                    + String.valueOf(secondPlaceDistance);

            // Robot.autonomousCommand[number] = new RobotDriveToTarget(2,
            // Constants.POSITION_RATE, 90, false, 3);
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To
            // CS";
            // number++;
            // Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place
            // Panel";
            break;

        case 4:
            secondPlaceDistance = 4;
            Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS "
                    + String.valueOf(Robot.sideAngle);
            number++;
            Robot.autonomousCommand[number] = new DriveAndPlacePanelShip(secondPlaceDistance, Constants.POSITION_RATE,
                    Robot.sideAngle, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                    + String.valueOf(secondPlaceDistance);

            // Robot.autonomousCommand[number] = new RobotDriveToTarget(8.,
            // Constants.POSITION_RATE, Robot.sideAngle,
            // false, 3);
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Drive To
            // CS";
            // number++;
            // Robot.autonomousCommand[number] = new PlaceHatchPanelShip();
            // Robot.autonomousCommandName[number] = String.valueOf(number) + " - Place
            // Panel";
            break;

        case 5:

            double rocketAngle = -28;
            if(Robot.invertY) rocketAngle = 28;
            secondPlaceDistance = 3;
            Robot.autonomousCommand[number] = new RobotOrient(rocketAngle, Constants.ORIENT_RATE,
                    true, 2);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To Rocket "
                    + String.valueOf(rocketAngle);
            number++;
            Robot.autonomousCommand[number] = new DriveAndPlacePanelRocket(secondPlaceDistance, Constants.POSITION_RATE,
                    Robot.sideAngle, false, 3);
            Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Rocket and Place "
                    + String.valueOf(secondPlaceDistance);
            break;
        }
        return number;
    }

    public static void updateStatus(int number) {
        for (int i = 0; i <= number; i++) {
            SmartDashboard.putString("AutoCommand " + String.valueOf(i), Robot.autonomousCommandName[i]);

        }

    }
}
