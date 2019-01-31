/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;

/**
 * Add your docs here.
 */
public class SetGyroOffset extends InstantCommand {
  /**
   * Add your docs here.
   */
  double myOffsetAngle;

  public SetGyroOffset(double offsetAngle) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myOffsetAngle = offsetAngle;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    DriveTrain.gyroOffset = myOffsetAngle;
  }

}
