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
import frc.robot.commands.TimeDelay;

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
                // side angle is set in Robot class + for left start, - for right
                // used to deliver hatches to side of cargo ship
                double firstPlaceDistance = 5;
                double positionRate = Pref.getPref("DrivePositionRate");
                int number = 1;
                Robot.autonomousCommand[number] = new PickAndRunTrajectory(true, true, Robot.invertY);
                Robot.autonomousCommandName[number] = "1 - Trajectory To CS1 Line";
                number++;
                Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true, 2);
                Robot.autonomousCommandName[number] = "2 - Orient to Cargo Ship " + String.valueOf(Robot.sideAngle);
                number++;
                Robot.autonomousCommand[number] = new DriveAndPlacePanelRocket(firstPlaceDistance, positionRate,
                                Robot.sideAngle, Robot.driveTrain.getRobotAtTarget(), 3);
                Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                                + String.valueOf(firstPlaceDistance);

                Robot.secondHatchPickupIndex = 0;
                Robot.secondHatchDeliverIndex = 1;

                return number;
        }

        public static int setMiddleStart() {
                double firstPlaceDistance = Pref.getPref("CenterDistance");
                double positionRate = Pref.getPref("DrivePositionRate");
                int number = 1;
                Robot.autonomousCommand[number] = new DriveAndPlacePanelRocket(firstPlaceDistance, positionRate, 0,
                Robot.driveTrain.getRobotAtTarget(), 3);
                Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                                + String.valueOf(firstPlaceDistance);

                Robot.secondHatchDeliverIndex = 0;
                return number;

        }

        public static int pickUpSecondHatch(int start, int indexStart) {
                int number = indexStart + 1;
                double pickupDistance = 12.5;
                double positionRate = Pref.getPref("DrivePositionRate");
                switch (start) {
                case 1:
                case 4:
                        Robot.autonomousCommand[number] = new BufferToActiveTrajectory(Robot.secondHatchPickupIndex);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Set Trajectory To LS";
                        number++;
                        Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, true, Robot.invertY);
                        Robot.autonomousCommandName[number] = String.valueOf(number)
                                        + " - Move to Load Station Trajectory";
                        number++;
                        Robot.autonomousCommand[number] = new RobotOrient(180, Constants.ORIENT_RATE, true, 1.5);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station 180";
                        number++;
                        Robot.autonomousCommand[number] = new DriveAndPickUpPanel(pickupDistance, positionRate, 180,
                                       Robot.driveTrain.getRobotAtTarget(), 2);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Load and Pickup "
                                        + String.valueOf(pickupDistance);
                        break;
                case 2:
                case 3:
                        pickupDistance = Pref.getPref("CenterLoadDistance");
                        Robot.autonomousCommand[number] = new PickAndRunTrajectory(false, true, Robot.invertY);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move to Load Trajectory";
                        number++;
                        Robot.autonomousCommand[number] = new RobotOrient(180, Constants.ORIENT_RATE, true, 3);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient to Load Station 180";
                        number++;
                        Robot.autonomousCommand[number] = new DriveAndPickUpPanel(pickupDistance, positionRate, 180,
                        Robot.driveTrain.getRobotAtTarget(), 3);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - To Load and PickUp "
                                        + String.valueOf(pickupDistance);

                        break;
                }
                return number;
        }

        // Hatch selected uses the int fron AutoChoosers, indexStart is the last command
        // from above

        public static int deliverSecondHatch(int hatchSelected, int indexStart) {
                double positionRate = Pref.getPref("DrivePositionRate");
                int number = indexStart;
                number++;
                // need to have gyro set for 0 to start trajectory
                // This means the side angle set in robot stays the same even thought the robot
                // now
                // moves in reverse
                Robot.autonomousCommand[number] = new SetGyroOffset(180);
                Robot.autonomousCommandName[number] = String.valueOf(number) + " - 180 Offset";
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

                        if (hatchSelected == 2)
                                secondPlaceDistance = 4;
                        Robot.autonomousCommand[number] = new RobotOrient(Robot.sideAngle, Constants.ORIENT_RATE, true,
                                        2);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS "
                                        + String.valueOf(Robot.sideAngle);
                        number++;
                        Robot.autonomousCommand[number] = new DriveAndPlacePanelRocket(secondPlaceDistance,
                                        positionRate, Robot.sideAngle, Robot.driveTrain.getRobotAtTarget(), 3);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
                                        + String.valueOf(secondPlaceDistance);

                        break;
                case 3:
                        double sideRocketAngle = 154;
                       if (Robot.startPositionSelected == 3 || Robot.startPositionSelected == 4)
                                sideRocketAngle = -154;

                        secondPlaceDistance = 5.;
                        Robot.autonomousCommand[number] = new RobotOrient(sideRocketAngle, Constants.ORIENT_RATE, true,
                                        2);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Orient To CS "
                                        + String.valueOf(sideRocketAngle);
                        number++;
                        Robot.autonomousCommand[number] = new DriveAndPlacePanelRocket(secondPlaceDistance,
                                        positionRate, sideRocketAngle, Robot.driveTrain.getRobotAtTarget(), 3);
                        Robot.autonomousCommandName[number] = String.valueOf(number) + " - Move To Ship and Place "
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
