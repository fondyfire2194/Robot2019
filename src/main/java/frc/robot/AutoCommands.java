/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Robot;

import frc.robot.commands.Auto.*;
import frc.robot.commands.Auto.CL1HabStart.*;
import frc.robot.commands.Auto.CR1HabStart.*;
import frc.robot.commands.Auto.LHab1Start.*;
import frc.robot.commands.Auto.RHab1Start.*;
import frc.robot.commands.*;
import frc.robot.commands.HatchPanels.*;

/**
 * Add your docs here.
 */
public class AutoCommands {

    public AutoCommands() {

    }

    public static int setLeftStart() {
        Robot.autonomousCommand[1] = new LHab1ToLCS2();
        Robot.autonomousCommand[2] = new LCToLoadApproach();
        Robot.autonomousCommand[3] = new RobotOrient(180, .5, true, 5);
        Robot.autonomousCommand[4] = new MoveToLoadStation();
        Robot.autonomousCommand[5] = new PickUpPanel();
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setLeftCenterStart() {
        Robot.autonomousCommand[1] = new LHab1ToLCS2();
        Robot.autonomousCommand[2] = new LCToLoadApproach();
        Robot.autonomousCommand[3] = new RobotOrient(180, .5, true, 5);
        Robot.autonomousCommand[4] = new MoveToLoadStation();
        Robot.autonomousCommand[5] = new PickUpPanel();
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setRightCenterStart(){
        Robot.autonomousCommand[1] = new LHab1ToLCS2();
        Robot.autonomousCommand[2] = new LCToLoadApproach();
        Robot.autonomousCommand[3] = new RobotOrient(180, .5, true, 5);
        Robot.autonomousCommand[4] = new MoveToLoadStation();
        Robot.autonomousCommand[5] = new PickUpPanel();
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setRightStart(){
        Robot.autonomousCommand[1] = new LHab1ToLCS2();
        Robot.autonomousCommand[2] = new LCToLoadApproach();
        Robot.autonomousCommand[3] = new RobotOrient(180, .5, true, 5);
        Robot.autonomousCommand[4] = new MoveToLoadStation();
        Robot.autonomousCommand[5] = new PickUpPanel();
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int secondHatchCommands(int hatchSelected, int indexStart) {
        int numberOfCommands = indexStart;
        switch (hatchSelected) {

        case 1:
            Robot.autonomousCommand[indexStart] = new BufferToActiveTrajectory(4);
            Robot.autonomousCommand[indexStart + 1] = new PathfinderTrajectory(true, false);
            break;

        case 2:

            break;
        case 3:

            break;
        }
        return numberOfCommands +5;
    }

}
