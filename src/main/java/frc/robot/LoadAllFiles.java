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

public class LoadAllFiles implements Runnable {
    public volatile boolean running = false;
    public volatile boolean error = false;

    public void run() {
        running = true;
        error = false;

        for (int i = 0; i < Robot.bufferTrajectoryName.length; i++) {
            Robot.bufferTrajectoryName[i] = "Not Used";
            SmartDashboard.putString("Buffer " + String.valueOf(i), Robot.bufferTrajectoryName[i]);
        }
        int pointer = 0;

        for (int i = 0; i < TrajDict.outsideStartNames.length; i++) {
            error = loadLeftFile(TrajDict.outsideStartNames[i], pointer);
            if (!error) {
                error = loadRightFile(TrajDict.outsideStartNames[i], pointer);
            }
            pointer++;
        }

        for (int i = 0; i < TrajDict.secondHatchPickupNames.length; i++) {
            error = loadLeftFile(TrajDict.secondHatchPickupNames[i], pointer);
            if (!error) {
                error = loadRightFile(TrajDict.secondHatchPickupNames[i], pointer);
            }

            pointer++;
        }
        SmartDashboard.putNumber("PTR", pointer);
        Robot.secondHatchIndex = TrajDict.secondHatchDeliveryNames.length;
        for (int i = 0; i < TrajDict.secondHatchDeliveryNames.length; i++) {
            error = loadLeftFile(TrajDict.secondHatchDeliveryNames[i], pointer);
            if (!error) {
                error = loadRightFile(TrajDict.secondHatchDeliveryNames[i], pointer);
            }
            pointer++;
        }

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
