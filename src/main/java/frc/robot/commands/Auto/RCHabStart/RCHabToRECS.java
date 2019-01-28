/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto.RCHabStart;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.TimeDelay;
import frc.robot.commands.Auto.SetAutoCommandDone;
import frc.robot.commands.RobotDriveToTarget;
import frc.robot.commands.HatchPanels.*;

public class RCHabToRECS extends CommandGroup {
  /**
   * Add your docs here.
   */
  public RCHabToRECS() {
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
    
    addSequential(new RobotDriveToTarget(10, .5, false, 10));//position using vision correction
        
    addSequential(new PlacePanel());
    addSequential(new SetAutoCommandDone(1));

    

  }
}