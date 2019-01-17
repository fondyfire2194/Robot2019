/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class BufferToActiveTrajectory extends InstantCommand {
  /**
   * Add your docs here.
   */
  public BufferToActiveTrajectory() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.activeTrajectory[0]=Robot.bufferTrajectory[0];
    Robot.activeTrajectory[1]=Robot.bufferTrajectory[1];
    Robot.activeTrajName = Robot.bufferTrajName;
    
  }

}
