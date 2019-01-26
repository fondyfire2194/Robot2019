/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.SecondPanel;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.BufferToActiveTrajectory;
import frc.robot.commands.PathfinderTrajectory;
import frc.robot.commands.RobotDriveToTarget;
import frc.robot.commands.RobotOrient;
import frc.robot.commands.Auto.SetAutoCommandDone;
import frc.robot.commands.HatchPanels.PlacePanel;

public class LoadToCS2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LoadToCS2(boolean side, int step) {
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

    addSequential(new BufferToActiveTrajectory(2));

    addSequential(new PathfinderTrajectory(Robot.faceField, side));
    if (side) {
      addSequential(new RobotOrient(-90, .5, false, 5));
    }
    if (!side) {
      addSequential(new RobotOrient(90, .5, false, 5));
    }

    addSequential(new RobotDriveToTarget(5, .5, false, 10));

    addSequential(new PlacePanel());

    addSequential(new SetAutoCommandDone(step));
  }
}
