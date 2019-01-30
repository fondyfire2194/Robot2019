/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.SecondPanel;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.Trajectories.BufferToActiveTrajectory;
import frc.robot.commands.Trajectories.PathfinderTrajectory;
import frc.robot.commands.Motion.RobotDriveToTarget;
import frc.robot.commands.Motion.RobotOrient;
import frc.robot.commands.Auto.SetAutoCommandDone;
import frc.robot.Robot;
public class LoadToECSNear extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LoadToECSNear(boolean side, int step) {
    // Add Commands here:
    // e.g. addSequential(new Command1());
    // addSequential(new Command2());
    // these will run in order.

    // To run multiple commands at the same time,
    // use addParallel()
    // e.g. addParallel(new Command1());
    // addSequential(new Command2());
    // Command1 and Command2 will run in parallel.

    // A command group will require all of the subsystems that each member
    // would require.
    // e.g. if Command1 requires chassis, and Command2 requires arm,
    // a CommandGroup containing them would require both the chassis and the
    // arm.

    addSequential(new BufferToActiveTrajectory(Robot.secondHatchIndex));
    addSequential(new PathfinderTrajectory(false, side));
    addSequential(new RobotOrient(0, .5, false, 3));
    addSequential(new RobotDriveToTarget(10, .5, false, 3));
    addSequential(new SetAutoCommandDone(step));


  }
}
