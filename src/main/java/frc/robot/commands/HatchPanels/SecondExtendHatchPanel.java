/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.HatchPanels;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

/**
 * Add your docs here.
 */
public class SecondExtendHatchPanel extends InstantCommand {
  /**
   * Add your docs here.
   */
  boolean myExtend;

  public SecondExtendHatchPanel(boolean extend) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myExtend = extend;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if (myExtend) {
      Robot.gph.secondExtendHatchPanel();
    } else {
      Robot.gph.secondRetractHatchPanel();
    }
  }

}
