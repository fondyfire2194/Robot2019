/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.Auto.SetAutoCommandDone;
import frc.robot.commands.HatchPanels.GripHatchPanel;
import frc.robot.commands.Elevator.SetElevatorTargetHeight;

public class AutoPickUpHatchPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoPickUpHatchPanel() {
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
    // addSequential(new ExtendHatchPanel(true));
    // addSequential(new TimeDelay(.5));
    addSequential(new SetElevatorTargetHeight(4.));
    addSequential(new TimeDelay(.7));
    addSequential(new GripHatchPanel(true));
    addSequential(new TimeDelay(.3));
    addSequential(new SetAutoCommandDone());
  }
}
