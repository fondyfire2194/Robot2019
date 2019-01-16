/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.LimelightControlMode.*;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class ToggleCamMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  public ToggleCamMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (Robot.limelightCamera.getCamMode() == CamMode.kdriver) {
      Robot.limelightCamera.setCamMode(CamMode.kvision);
    } else {
      Robot.limelightCamera.setCamMode(CamMode.kdriver);
    }
  }

}
