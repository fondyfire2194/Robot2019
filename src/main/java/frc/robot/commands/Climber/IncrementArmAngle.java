/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
/**
 * Add your docs here.
 */
public class IncrementArmAngle extends InstantCommand {
  /**
   * Add your docs here.
   */
  double myIncrement;
  public IncrementArmAngle(double increment) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myIncrement = increment;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.climberArm.armTargetDegrees+= myIncrement;
    if (Robot.climberArm.armTargetDegrees > 110) {
      Robot.climberArm.armTargetDegrees = 110;
    }
    if (Robot.climberArm.armTargetDegrees < 0) {
      Robot.climberArm.armTargetDegrees = 0;
    }


  }

}
