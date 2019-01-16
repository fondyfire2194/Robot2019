/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.SD;

/**
 * Add your docs here.
 */
public class Limelight2194 {

    public Limelight2194() {
    }

    public void updateStatus() {
        SmartDashboard.putBoolean("VisionTarget", Robot.limelightCamera.getIsTargetFound());
        SD.putN2("DegToTarget", Robot.limelightCamera.getdegRotationToTarget());
        SD.putN2("TargetArea", Robot.limelightCamera.getTargetArea());
        SD.putN2("BNDBoxWidth", Robot.limelightCamera.getBoundingBoxWidth());
        SD.putN2("BndBoxHeight", Robot.limelightCamera.getBoundingBoxHeight());
        SD.putN1("Pipeline Latency", Robot.limelightCamera.getPipelineLatency());

    }
}
