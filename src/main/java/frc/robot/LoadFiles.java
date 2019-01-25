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
import java.util.Arrays;

public class LoadFiles implements Runnable {
    public boolean valid = true;
    public boolean done = false;
    public boolean running = false;
    private String[] startNames;
    

    public void run() {
        running = true;
        for (int a = 0; a < Robot.bufferTrajectoryName.length; a++) {
            Robot.bufferTrajectoryName[a] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(a), Robot.bufferTrajectoryName[a]);
        }
        Robot.secondHatchIndex=0;
        int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
        SmartDashboard.putNumber("LGTH",0);
        if(startPositionSelected!=0){
        startNames=getTraj(startPositionSelected);
        SmartDashboard.putNumber("LGTH",startNames.length);
        int i = 0;
        while (valid && i < startNames.length) {

            loadLeftFile(startNames[i], i);
            try {
                Thread.sleep(2);
            } catch (InterruptedException ex) {

            }
            loadRightFile(startNames[i], i);
            try {
                Thread.sleep(2);
            } catch (InterruptedException ex) {

            }

            i++;

        }
        int secondHatchChosen = AutoChoosers.secondHatchChooser.getSelected();
        if(secondHatchChosen!=0)
        loadSecondHatchFile(secondHatchChosen,i);
       
    }
        done = true;
        Robot.readingRunning = false;
        // valid = false;
        Robot.startSettingsDone = true;
        Robot.startSettingsReady = false;

    }

    void loadLeftFile(String startName, int i) {
        Robot.leftBufferTrajectory[i] = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        Robot.bufferTrajectoryName[i] = startName;
        SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);

    }

    void loadRightFile(String startName, int i) {
        Robot.rightBufferTrajectory[i] = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);
        Robot.bufferTrajectoryName[i] = startName;
        SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);

    }

    void loadSecondHatchFile(int secondHatchChosen, int number) {
        
        String name = TrajDict.secondHatchNames[secondHatchChosen];
        Robot.leftBufferTrajectory[number] = BuildTrajectory.buildLeftFileName(Robot.useUsb, name);
        Robot.rightBufferTrajectory[number] = BuildTrajectory.buildRightFileName(Robot.useUsb, name);
        Robot.bufferTrajectoryName[number] = name;
        SmartDashboard.putString("Buffer " + String.valueOf(number), Robot.bufferTrajectoryName[number]);

        Robot.secondHatchIndex = number;

    }

    String[] getTraj(int startPositionSelected) {
       
        String[] startNames;
        switch (startPositionSelected) {
        case 0:
            startNames = TrajDict.leftStartNames;
            break;
        case 1:
            startNames = TrajDict.leftCenterStartNames;
            break;
        case 2:
            startNames = TrajDict.rightCenterStartNames;
            break;
        case 3:
            startNames = TrajDict.rightStartNames;
            break;
        default:
            throw new RuntimeException("need to code more selections");
        }
        return startNames;
    }

}
