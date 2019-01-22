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
public class SetLimelightLeds extends InstantCommand {
  /**
   * Add your docs here.
   */
  private LedMode myLedMode;
  public SetLimelightLeds( LedMode ledMode){
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myLedMode = ledMode;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.limelightCamera.setLEDMode(myLedMode);
  }

}
