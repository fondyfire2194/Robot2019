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
    private String[] startNames;

    public void run() {
        running = true;
        error = false;
        for (int i = 0; i < Robot.bufferTrajectoryName.length; i++) {
            Robot.bufferTrajectoryName[i] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
        }

        Robot.secondHatchIndex = 0;
        int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
        if (startPositionSelected < 0 || startPositionSelected > 4)
            startPositionSelected = 0;
        SmartDashboard.putNumber("LGTH", 0);
        if (startPositionSelected != 0) {
            startNames = getTraj(startPositionSelected);

            int i = 0;

            while (running && !error && i < startNames.length) {

                error = loadLeftFile(startNames[i], i);
                SmartDashboard.putBoolean("ERR", error);

                if (!error) {
                    error = loadRightFile(startNames[i], i);
                    i++;
                }
            }
            if (!error) {
                int secondHatchChosen = AutoChoosers.secondHatchChooser.getSelected();
                if (secondHatchChosen < 0 || secondHatchChosen > 5)
                    secondHatchChosen = 0;
                if (secondHatchChosen != 0) {
                    error = loadLeftSecondHatchFile(secondHatchChosen, i);
                    if (!error)
                        error = loadRightSecondHatchFile(secondHatchChosen, i);
                }
            }
        }
        running = false;
    }

    boolean loadLeftFile(String startName, int i) {
        Robot.leftBufferTrajectory[i] = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        if (Robot.buildOK) {
            Robot.bufferTrajectoryName[i] = startName;
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
            SmartDashboard.putNumber("BufferL Lngth" + String.valueOf(i), Robot.leftBufferTrajectory[i].length());

        }
        return !Robot.buildOK;
    }

    boolean loadRightFile(String startName, int i) {

        Robot.rightBufferTrajectory[i] = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);

        if (Robot.buildOK) {
            Robot.bufferTrajectoryName[i] = startName;
            Robot.bufferTrajectoryGains[i] = TrajDict.getTrajGains(startName);
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
            SmartDashboard.putNumber("Buffer R Lngth" + String.valueOf(i), Robot.rightBufferTrajectory[i].length());

        }
        return !Robot.buildOK;
    }

    boolean loadLeftSecondHatchFile(int secondHatchChosen, int number) {

        String name = TrajDict.secondHatchNames[secondHatchChosen];
        Robot.leftBufferTrajectory[number] = BuildTrajectory.buildLeftFileName(Robot.useUsb, name);
        Robot.bufferTrajectoryGains[number] = TrajDict.getTrajGains(name);
        Robot.bufferTrajectoryName[number] = name;
        SmartDashboard.putString("Buffer " + String.valueOf(number), Robot.bufferTrajectoryName[number]);
        SmartDashboard.putNumber("Buffer L Lngth" + String.valueOf(number), Robot.leftBufferTrajectory[number].length());

        Robot.secondHatchIndex = number;
        return !Robot.buildOK;

    }

    boolean loadRightSecondHatchFile(int secondHatchChosen, int number) {

        String name = TrajDict.secondHatchNames[secondHatchChosen];
        Robot.rightBufferTrajectory[number] = BuildTrajectory.buildRightFileName(Robot.useUsb, name);
        Robot.bufferTrajectoryName[number] = name;
        SmartDashboard.putString("Buffer " + String.valueOf(number), Robot.bufferTrajectoryName[number]);
        SmartDashboard.putNumber("Buffer R Lngth" + String.valueOf(number), Robot.rightBufferTrajectory[number].length());

        Robot.secondHatchIndex = number;
        return !Robot.buildOK;

    }

    String[] getTraj(int startPositionSelected) {

        String[] startNames;
        switch (startPositionSelected) {
        case 1:
            startNames = TrajDict.leftStartNames;
            Robot.numberOfAutonomousCommands = AutoCommands.setLeftStart();
            break;
        case 2:
            startNames = TrajDict.leftCenterStartNames;
            Robot.numberOfAutonomousCommands = AutoCommands.setLeftCenterStart();
            break;
        case 3:
            startNames = TrajDict.rightCenterStartNames;
            Robot.numberOfAutonomousCommands = AutoCommands.setRightCenterStart();
            break;
        case 4:
            startNames = TrajDict.rightStartNames;
            Robot.numberOfAutonomousCommands = AutoCommands.setRightStart();
            break;
        default:
            throw new RuntimeException("need to code more selections");
        }
        return startNames;
    }

}
