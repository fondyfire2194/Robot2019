/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.command.InstantCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.Timer;
/**
 * Add your docs here.
 */
public class BufferToActiveTrajectory extends InstantCommand {
  /**
   * Add your docs here.
   */
  private int myNumber;
  public BufferToActiveTrajectory(int number) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myNumber=number;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    SmartDashboard.putNumber("MyN",myNumber);
    Robot.activeLeftTrajectory= Robot.leftBufferTrajectory[myNumber];
     Robot.activeRightTrajectory = Robot.rightBufferTrajectory[myNumber];
     Robot.activeTrajectoryGains = Robot.bufferTrajectoryGains[myNumber];
     Robot.activeTrajName = Robot.bufferTrajectoryName[myNumber];
     SmartDashboard.putNumber("A2BL", Robot.activeLeftTrajectory.length());

Robot.autonomousCommandDone=true;

  }

}
