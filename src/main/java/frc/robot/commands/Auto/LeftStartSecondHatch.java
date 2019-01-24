/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;

public class LeftStartSecondHatch extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LeftStartSecondHatch(int number) {
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

    switch (number) {
      case 0:
        // teleop
        break;

      case 1:
        // Left cargo ship 1  addSequential
        break;

      case 2:
        // left cargo ship 2
        break;

      case 3:
        // left cargo ship 3
        break;
        
      case 4:
        // end cargo ship left
        break;

      case 5:
        // end cargo ship right
        break;
      
      default:
        break;
    }
  }
}
