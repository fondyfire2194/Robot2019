/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.AutoChoosers;

/**
 * Vision validity check while in motion. Vision distance should change the
 * san=me amount as encoder distance.
 */
public class VisionData {

    public int[] boxWidth;
    public double[] distanceFeet;
    private double goodMinWidth = 50;// 6.7 ft
    private double goodMaxWidth = 80;// 3.8 ft

    public VisionData() {
        boxWidth = new int[] { 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 140, 145,
                150, 155, 160, 165, 170 };
        distanceFeet = new double[] { 8.0, 6.7, 6.2, 5.5, 5.0, 4.6, 4.25, 3.8, 3.5, 3.3, 3.0, 2.9, 2.65, 2.5, 2.43,
                2.25, 2.1, 2.0, 1.95, 1.88, 1.84, 1.8, 1.75, 1.73, 1.7 };
    }

    public double calculateCameraDistance() {
        int lookUpIndex = 0;
        int segmentRange = 5;
        int distIntoRange = 0;
        double distanceRange = 0;
        double boxWidthValue = Robot.limelightCamera.getBoundingBoxWidth();
        if (boxWidthValue >= 45 && boxWidthValue < 160) {
            lookUpIndex = (int) (boxWidthValue - boxWidth[0]) / 5;
            distIntoRange = ((int) boxWidthValue - boxWidth[0] - lookUpIndex * 5);
            double intoRange = (double) distIntoRange / segmentRange;
            distanceRange = (distanceFeet[lookUpIndex] - distanceFeet[lookUpIndex + 1]);
            return (distanceFeet[lookUpIndex] - distanceRange * intoRange);
        } else
            return 0;
    }

    public double getRobotVisionDistance() {
        return calculateCameraDistance() - Constants.CAMERA_TO_FRONT_OF_BUMPER;
    }

    public boolean inGoodVisionDistanceRange() {
        double widthSeen = Robot.limelightCamera.getBoundingBoxWidth();
        return Robot.limelightCamera.getIsTargetFound() && (widthSeen < goodMaxWidth && widthSeen > goodMinWidth);
    }

    public void updateStatus() {
        SD.putN1("RobotToTargetFt", getRobotVisionDistance());

        if (AutoChoosers.debugChooser.getSelected() == 7) {
            SD.putN1("TargetDistance", calculateCameraDistance());

        }
    }

}
