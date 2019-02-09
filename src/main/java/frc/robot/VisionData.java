/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import jaci.pathfinder.Pathfinder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class VisionData {

    public int[] boxHeight;
    public double[] distanceFeet;
    public double[] validTargetAngles;
    private double atTargetAngle = 5.;

    public VisionData() {
        boxHeight = new int[] { 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90 };

        distanceFeet = new double[] { 10, 9, 8, 7, 6.1, 5.5, 4.1, 3.5, 3.2, 2.8, 2.5, 2.3, 2.7, 2.4, 1.9, 1.8, 1.3, 1.2,
                1.0 };

        validTargetAngles = new double[] { 0, 62.5, 90, 152.5, 180, -152.5, -90, -62.5 };

    }

    public double calculateDistance() {
        int lookUpIndex = 0;
        int segmentRange = 0;
        int distIntoRange = 0;
        double distanceRange = 0;

        double boxHeightValue = Robot.limelightCamera.getBoundingBoxHeight();
        if (boxHeightValue > 0 && boxHeightValue < 85) {
            lookUpIndex = (int) boxHeightValue / 5;
            segmentRange = (boxHeight[lookUpIndex + 1] - boxHeight[lookUpIndex]);
            if (segmentRange != 0) {
                distIntoRange = ((int) boxHeightValue - lookUpIndex * 5);
                distanceRange = (distanceFeet[lookUpIndex] - distanceFeet[lookUpIndex + 1]);
                return (distanceFeet[lookUpIndex] - ((distanceRange * (double) distIntoRange) / (double) segmentRange));
            } else
                return 0;
        } else
            return 0;

    }

    public double atTargetAngle() {
        double temp = 999;
        boolean angleFound = false;
        int i = -1;
        while (i < validTargetAngles.length - 1 && !angleFound) {
            i++;
            angleFound = Math.abs(Robot.driveTrain.getGyroYaw() - validTargetAngles[i]) < atTargetAngle;
            if (angleFound) {
                temp = validTargetAngles[i];
                break;
            }
        }
        return temp;
    }
    public double getGyroAngleError(){

        return atTargetAngle - Robot.driveTrain.getGyroYaw();
    }

    public void updateStatus() {
        SD.putN1("TargetDistance", calculateDistance());
        SmartDashboard.putBoolean("AtTargetAngle", atTargetAngle() != 999);

    }

}
