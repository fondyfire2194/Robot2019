/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Limelight;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SetPipeline extends InstantCommand {
  /**
   * Pipelines available
   * 0 - all Leds
   * 1 - left (top) leds
   * 2 - right (bottom) leds
   */
  private int myNumber;

  public SetPipeline(int number) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myNumber = number;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (myNumber >= 0 || myNumber <= 9)
      Robot.limelightCamera.setPipeline(myNumber);
  }

}
