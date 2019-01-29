/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.LCHabStart;

import edu.wpi.first.wpilibj.command.CommandGroup;

import frc.robot.commands.Motion.RobotDriveToTarget;
import frc.robot.commands.HatchPanels.*;
import frc.robot.commands.Auto.*;

public class LCHabToLECS extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LCHabToLECS() {
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

    /*
     * Pathfinder always generates X moves starting at 0 in the plus direction from
     * alliance wall toward the field. We can run the robot starting at the
     * beginning or end of the generated trajectory leading with the front or rear
     * of the robot. PathfinderTrajectory starts at the beginning of the trajectory
     * and PathfinderReverseTrajectory from the end. A True entry means we want the
     * front of the robot to lead and a False means we want the rear to lead.
     */

    /*
     * Center start opposite left hatch. Position using vision. Attach panel. Build
     * trajectory for next group
     * 
     * 
     * 
     */

    addSequential(new SetRunningCommandName("Vision Move"));
    addSequential(new RobotDriveToTarget(10, .5, false, 10));// position using vision correction
    addSequential(new SetRunningCommandName("Place Panel"));
    addSequential(new PlaceHatchPanel());

    addSequential(new SetAutoCommandDone(1));

  }
}
