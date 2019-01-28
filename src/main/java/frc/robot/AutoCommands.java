/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Robot;

import frc.robot.commands.Auto.*;
import frc.robot.commands.Auto.LCHabStart.*;
import frc.robot.commands.Auto.RCHabStart.*;
import frc.robot.commands.Auto.LHabStart.*;
import frc.robot.commands.Auto.RHabStart.*;
import frc.robot.commands.Auto.SecondPanel.LoadToCS1;
import frc.robot.commands.Auto.SecondPanel.LoadToCS2;
import frc.robot.commands.Auto.SecondPanel.LoadToCS3;
import frc.robot.commands.Auto.SecondPanel.LoadToECSFar;
import frc.robot.commands.Auto.SecondPanel.LoadToECSNear;
import frc.robot.commands.*;
import frc.robot.commands.HatchPanels.*;

/**
 * Add your docs here.
 */
public class AutoCommands {

    public AutoCommands() {

    }

    public static int setLeftStart() {
        Robot.autonomousCommand[1] = new LHabToLCS2();
        Robot.autonomousCommand[2] = new LCS2ToLLoad();
        Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 2, 3);
        Robot.autonomousCommand[4] = new MoveToLoadStation(4);
        Robot.autonomousCommand[5] = new PickUpPanel(5);
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setLeftCenterStart() {
        Robot.autonomousCommand[1] = new LCHabToLECS();
        Robot.autonomousCommand[2] = new LCToLLoad();
        Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 5, 3);
        Robot.autonomousCommand[4] = new MoveToLoadStation(4);
        Robot.autonomousCommand[5] = new PickUpPanel(5);
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setRightCenterStart(){
        Robot.autonomousCommand[1] = new RCHabToRECS();
        Robot.autonomousCommand[2] = new RCToRLoad();
        Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 5, 3);
        Robot.autonomousCommand[4] = new MoveToLoadStation(4);
        Robot.autonomousCommand[5] = new PickUpPanel(5);
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    public static int setRightStart(){
        Robot.autonomousCommand[1] = new RHabToRCS2();
        Robot.autonomousCommand[2] = new RCS2ToRLoad();
        Robot.autonomousCommand[3] = new AutonomousOrient(180, .5, true, 5, 3);
        Robot.autonomousCommand[4] = new MoveToLoadStation(4);
        Robot.autonomousCommand[5] = new PickUpPanel(5);
        int numberOfCommands = 5;
        return numberOfCommands;
    }

    // Hatch selected uses the int fron AutoChoosers, indexStart is the last command from above
    // Side is true if the starting position is right, use to make Robot.invertY true
    public static int secondHatchCommands(int hatchSelected, int indexStart, boolean side) {
        int numberOfCommands = indexStart;
        switch (hatchSelected) {
        
        case 1:
            Robot.autonomousCommand[indexStart + 1] = new LoadToCS1(side, indexStart + 1);
            break;
        case 2:
            Robot.autonomousCommand[indexStart + 1] = new LoadToCS2(side, indexStart + 1);
            break;
        case 3:
            Robot.autonomousCommand[indexStart + 1] = new LoadToCS3(side, indexStart + 1);
            break;
        case 4:
            Robot.autonomousCommand[indexStart + 1] = new LoadToECSNear(side, indexStart + 1);
            break;
        case 5:
            Robot.autonomousCommand[indexStart + 1] = new LoadToECSFar(side, indexStart + 1);
            break;
        }
        return numberOfCommands +1;
    }

}
