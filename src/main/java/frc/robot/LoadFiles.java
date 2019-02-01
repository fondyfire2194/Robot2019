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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.TrajDict;

public class LoadFiles implements Runnable {
    public volatile boolean running = false;
    public volatile boolean error = false;

    public void run() {
        running = true;
        error = false;
        for (int i = 0; i < Robot.bufferTrajectoryName.length; i++) {
            Robot.bufferTrajectoryName[i] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
        }

        int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
        int i = 0;
        switch (startPositionSelected) {
        case 0:
        case 2:
        case 3:
            break;
        case 1:
        case 4:
            error = loadLeftFile(TrajDict.outsideStartNames[0], i);
            if (!error) {
                error = loadRightFile(TrajDict.outsideStartNames[0], i);
            }
            i++;
            break;
        }
        int secondHatchChosen = AutoChoosers.secondHatchChooser.getSelected();
        if (startPositionSelected != 0 && secondHatchChosen != 0) {

            int startPositionSelectedIndex = 0;
            if (startPositionSelected == 2 || startPositionSelected == 3)
                startPositionSelectedIndex = 1;

            error = loadLeftFile(TrajDict.secondHatchPickupNames[startPositionSelectedIndex], i);
            if (!error)
                error = loadRightFile(TrajDict.secondHatchPickupNames[startPositionSelectedIndex], i);

            i++;

            error = loadLeftFile(TrajDict.secondHatchDeliveryNames[secondHatchChosen - 1], i);
            if (!error) {
                error = loadRightFile(TrajDict.secondHatchDeliveryNames[secondHatchChosen - 1], i);
            }
        }
        Robot.secondHatchIndex = i;
        running = false;

    }

    boolean loadLeftFile(String startName, int i) {
        Robot.buildOK = false;
        Robot.leftBufferTrajectory[i] = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        if (Robot.buildOK) {
            Robot.bufferTrajectoryName[i] = startName;
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
            SmartDashboard.putNumber("BufferL Lngth" + String.valueOf(i), Robot.leftBufferTrajectory[i].length());

        }
        return !Robot.buildOK;
    }

    boolean loadRightFile(String startName, int i) {
        Robot.buildOK = false;
        Robot.rightBufferTrajectory[i] = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);

        if (Robot.buildOK) {
            Robot.bufferTrajectoryName[i] = startName;
            Robot.bufferTrajectoryGains[i] = TrajDict.getTrajGains(startName);
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
            SmartDashboard.putNumber("Buffer R Lngth" + String.valueOf(i), Robot.rightBufferTrajectory[i].length());

        }
        return !Robot.buildOK;
    }

}
