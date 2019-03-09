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
        Robot.activeTrajName = "Not Used";
        SmartDashboard.putString("Active Trajectory", Robot.activeTrajName);
        SmartDashboard.putNumber("ActTrajLngth", 0);

        for (int i = 0; i < Robot.bufferTrajectoryName.length; i++) {
            Robot.bufferTrajectoryName[i] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
            SmartDashboard.putNumber("BufferL Lngth" + String.valueOf(i), 0);
            SmartDashboard.putNumber("BufferR Lngth" + String.valueOf(i), 0);
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
        if (!error && startPositionSelected != 0 && secondHatchChosen != 0) {

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

        running = false;

    }

    boolean loadLeftFile(String startName, int i) {

        Robot.buildOK = false;
        if (i == 0) {
            Robot.activeLeftTrajectory = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        } else {
            Robot.leftBufferTrajectory[i - 1] = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        }
        if (Robot.buildOK && i != 0) {
            Robot.bufferTrajectoryName[i - 1] = startName;
            SmartDashboard.putString("Buffer " + String.valueOf(i - 1), Robot.bufferTrajectoryName[i - 1]);
            SmartDashboard.putNumber("BufferL Lngth" + String.valueOf(i - 1),
                    Robot.leftBufferTrajectory[i - 1].length());

        }
        return !Robot.buildOK;
    }

    boolean loadRightFile(String startName, int i) {
        Robot.buildOK = false;
        if (i == 0) {
            Robot.activeRightTrajectory = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);
        } else {
            Robot.rightBufferTrajectory[i - 1] = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);
        }

        if (Robot.buildOK) {
            if (i == 0) {
                Robot.activeTrajName = startName;
                int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
                
                    Robot.activeTrajectoryGains = TrajDict.getTrajGains(startName);
                

                SmartDashboard.putString("Active Trajectory", Robot.activeTrajName);
                SmartDashboard.putNumber("ActTrajLngth", Robot.activeLeftTrajectory.length());
            } else {
                Robot.bufferTrajectoryName[i - 1] = startName;
                int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
                
                    Robot.bufferTrajectoryGains[i - 1] = TrajDict.getTrajGains(startName);
        
                SmartDashboard.putString("Buffer " + String.valueOf(i - 1), Robot.bufferTrajectoryName[i - 1]);
                SmartDashboard.putNumber("Buffer R Lngth" + String.valueOf(i - 1),
                        Robot.rightBufferTrajectory[i - 1].length());
            }
        }
        return!Robot.buildOK;
}

}
