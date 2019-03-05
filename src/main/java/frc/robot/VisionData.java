/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.AutoChoosers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Pref;

/**
 * Add your docs here.
 */
public class VisionData {

    public int[] boxWidth;
    public double[] distanceFeet;

    public VisionData() {
        boxWidth = new int[] { 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 140,
                145 };

        distanceFeet = new double[] { 7.9, 7.04, 6.1, 5.38, 4.75, 4.4, 3.82, 3.5, 3.13, 2.75, 2.62, 2.35, 2.17, 2.01,
                1.8, 1.7, 1.55, 1.4, 1.25, 1.15, 1.1, 1 };
    }

    public double calculateDistance() {
        int lookUpIndex = 0;
        int segmentRange = 5;
        int distIntoRange = 0;
        double distanceRange = 0;
        double boxWidthValue = Robot.limelightCamera.getBoundingBoxWidth();
        if (boxWidthValue >= 40 && boxWidthValue < 145) {
            lookUpIndex = (int) (boxWidthValue - boxWidth[0]) / 5;
            distIntoRange = ((int) boxWidthValue - boxWidth[0] - lookUpIndex * 5);
            double intoRange = (double) distIntoRange / segmentRange;
            distanceRange = (distanceFeet[lookUpIndex] - distanceFeet[lookUpIndex + 1]);
            return (distanceFeet[lookUpIndex] - distanceRange * intoRange);
        } else
            return 0;
    }

    public void updateStatus() {
        SD.putN1("TargetDistance", calculateDistance());

        if (AutoChoosers.debugChooser.getSelected() == 7) {

        }
    }

}
