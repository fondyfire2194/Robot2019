/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;

int myNumber;

/**
 * Add your docs here.
 */
public class SetAutoCommandDone extends InstantCommand {
  /**
   * Add your docs here.
   */
  public SetAutoCommandDone(int number) {
    myNumber = number;
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {

    Robot.autonomousCommandDone[myNumber] = true;

  }

}
