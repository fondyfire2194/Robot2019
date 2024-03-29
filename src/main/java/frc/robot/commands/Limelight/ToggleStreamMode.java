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
public class ToggleStreamMode extends InstantCommand {
  /**
   * Add your docs here.
   */
  StreamType type;

  public ToggleStreamMode() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    type = Robot.limelightCamera.getStream();
    switch (type) {
    case kStandard:
      Robot.limelightCamera.setStream(StreamType.kPiPMain);
      break;
    case kPiPMain:
      Robot.limelightCamera.setStream(StreamType.kPiPSecondary);
      break;
    case kPiPSecondary:
      Robot.limelightCamera.setStream(StreamType.kStandard);
      break;
    default:
      Robot.limelightCamera.setStream(StreamType.kStandard);
      break;
    }

  }

}
