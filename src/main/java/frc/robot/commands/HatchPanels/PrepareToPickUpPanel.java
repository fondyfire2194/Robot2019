/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanels;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;
import frc.robot.commands.HatchPanels.ExtendHatchPanel;
import frc.robot.commands.HatchPanels.PushHatchPanel;
import frc.robot.commands.HatchPanels.GripHatchPanel;
import frc.robot.commands.HatchPanels.PushHatchPanel;

public class PrepareToPickUpPanel extends CommandGroup {
  /**
   * Add your docs here.
   */
  public PrepareToPickUpPanel() {
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
    addParallel(new GripHatchPanel(false));
    addParallel(new ExtendHatchPanel(false));
    addParallel(new SecondExtendHatchPanel(false)); 
    addParallel(new PushHatchPanel(false));


  }
}
