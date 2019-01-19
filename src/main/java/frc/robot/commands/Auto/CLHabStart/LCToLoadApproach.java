/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.CLHabStart;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.PathfinderReverseTrajectory;
import frc.robot.commands.PathfinderTrajectory;
import frc.robot.commands.RobotDriveToTarget;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.HatchPanels.*;
import frc.robot.commands.BuildTrajectoryToBuffer;
import frc.robot.Robot;
import frc.robot.commands.BufferToActiveTrajectory;

public class LCToLoadApproach extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LCToLoadApproach() {
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
    /**
     * Two curves to get C shape, Pathfinder can't change X direction Do reverse
     * curve back and up Do reverse curve up and forward Do curve to load station
     * Pick up panel Do reverse curve forward and left Position using vision Attach
     * panel
     */


    addSequential(new BufferToActiveTrajectory());//rev to wall and left

    addParallel(new BuildTrajectoryToBuffer(Robot.useUsb, "left and rev to field"));// left and rev to field

    addSequential(new PathfinderReverseTrajectory(Robot.faceField,Robot.invertY));// rev to wall and left

    addSequential(new BufferToActiveTrajectory());//left and rev to field

    addParallel(new BuildTrajectoryToBuffer(Robot.useUsb, ""));// to load approach

    addSequential(new PathfinderTrajectory(!Robot.faceField,!Robot.invertY));// left and reverse to field

    addSequential(new BufferToActiveTrajectory());//rev to wall and left

 }
}
