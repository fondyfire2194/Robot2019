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
public class VisionData {

    public int[] boxHeight;
    public double[] distanceFeet;

    public VisionData() {
        boxHeight = new int[] { 0, 0, 0, 0, 0, 23, 25, 30, 35, 40, 45, 50, 55, 60, 65, 71, 77, 80, 86, 90, 97, 0, 0, 0,
                0, 0, 0, 0 };

        distanceFeet = new double[] { 0, 0, 0, 0, 0, 89.5, 82.75, 64, 51, 43.6, 35.7, 30.9, 27.4, 24.2, 21.1, 20, 15.7,
                14, 13.3, 12.7, 9.8, 0, 0, 0, 0, 0, 0, 0 };

    }

    public double calculateDistance() {
        int lookUpIndex=0;
        int segmentRange = 0;
        int distIntoRange =0;
        double distanceRange=0;
        double boxHeightValue = Robot.limelightCamera.getBoundingBoxHeight();
        if ( boxHeightValue> 0 && boxHeightValue < 130) {
            lookUpIndex = (int)(int)boxHeightValue / 5;
            segmentRange = (boxHeight[lookUpIndex + 1] - boxHeight[lookUpIndex]);
            if (segmentRange != 0) {
                distIntoRange = ((int)boxHeightValue - lookUpIndex * 5);
                distanceRange = (distanceFeet[lookUpIndex] - distanceFeet[lookUpIndex + 1]);
                return (distanceFeet[lookUpIndex]
                        - ((distanceRange * (double) distIntoRange) / (double) segmentRange));
            } else
                return 0;
        } else
            return 0;

    }

}
