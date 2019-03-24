/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class SetClimberTargetAngle extends InstantCommand {
  /**
   * Add your docs here.
   */
  private double myDegrees;
  private boolean myClimb;
  public SetClimberTargetAngle(double degrees, boolean climb) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myDegrees = degrees;
    myClimb = climb;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    if(myClimb){
      Robot.climberArm.motionMagicRate = Constants.CLIMBER_ARM_CLIMB_RATE;
    }
    else{
      Robot.climberArm.motionMagicRate = Constants.CLIMBER_ARM_POSITION_RATE;
  
    }
    Robot.climberArm.armTargetDegrees = myDegrees;
  }

}
