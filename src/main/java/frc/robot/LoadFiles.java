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

public class LoadFiles implements Runnable {
    public volatile boolean running = false;
    public volatile boolean error = false;
    private String[] startNames;

    public void run() {
        running = true;
        error = false;
        for (int a = 0; a < Robot.bufferTrajectoryName.length; a++) {
            Robot.bufferTrajectoryName[a] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(a), Robot.bufferTrajectoryName[a]);
        }
        Robot.secondHatchIndex = 0;
        int startPositionSelected = AutoChoosers.startPositionChooser.getSelected();
        SmartDashboard.putNumber("LGTH", 0);
        if (startPositionSelected != 0) {
            startNames = getTraj(startPositionSelected);

            int i = 0;

            while (running && !error && i < startNames.length) {

                error = loadLeftFile(startNames[i], i);
                try {
                    Thread.sleep(2);
                } catch (InterruptedException ex) {

                }
                if (error)
                    break;
                error = loadRightFile(startNames[i], i);
                if (error)
                    break;
                try {
                    Thread.sleep(2);
                } catch (InterruptedException ex) {

                }

                i++;

                int secondHatchChosen = AutoChoosers.secondHatchChooser.getSelected();
                if (secondHatchChosen != 0) {
                    error = loadLeftSecondHatchFile(secondHatchChosen, i);
                    if (error)
                        break;
                    error = loadRightSecondHatchFile(secondHatchChosen, i);
                }
            }
        }
        running = false;
    }

    boolean loadLeftFile(String startName, int i) {
        Robot.leftBufferTrajectory[i] = BuildTrajectory.buildLeftFileName(Robot.useUsb, startName);
        Robot.bufferTrajectoryName[i] = startName;
        SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
        return !Robot.buildOK;
    }

    boolean loadRightFile(String startName, int i) {
        Robot.rightBufferTrajectory[i] = BuildTrajectory.buildRightFileName(Robot.useUsb, startName);
        Robot.bufferTrajectoryName[i] = startName;
        SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
        return !Robot.buildOK;
    }

    boolean loadLeftSecondHatchFile(int secondHatchChosen, int number) {

        String name = TrajDict.secondHatchNames[secondHatchChosen];
        Robot.leftBufferTrajectory[number] = BuildTrajectory.buildLeftFileName(Robot.useUsb, name);
        Robot.bufferTrajectoryName[number] = name;
        SmartDashboard.putString("Buffer " + String.valueOf(number), Robot.bufferTrajectoryName[number]);

        Robot.secondHatchIndex = number;
        return !Robot.buildOK;

    }

    boolean loadRightSecondHatchFile(int secondHatchChosen, int number) {

        String name = TrajDict.secondHatchNames[secondHatchChosen];
        Robot.rightBufferTrajectory[number] = BuildTrajectory.buildRightFileName(Robot.useUsb, name);
        Robot.bufferTrajectoryName[number] = name;
        SmartDashboard.putString("Buffer " + String.valueOf(number), Robot.bufferTrajectoryName[number]);

        Robot.secondHatchIndex = number;
        return !Robot.buildOK;

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
