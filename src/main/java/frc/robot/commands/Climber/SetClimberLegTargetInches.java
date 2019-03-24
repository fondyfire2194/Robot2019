/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Climber;

import frc.robot.Robot;
import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Constants;

/**
 * Add your docs here.
 */
public class SetClimberLegTargetInches extends InstantCommand {
  /**
   * Add your docs here.
   */
  double myTarget;
  boolean myClimb;
  public SetClimberLegTargetInches(double target, boolean climb) {
    super();
    
    myTarget = target;
    myClimb = climb;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override 
  protected void initialize() {
    if(myClimb){
       Robot.climberLeg.motionMagicRate = Constants.CLIMBER_LEG_CLIMB_RATE;     
    }
    else{
      Robot.climberLeg.motionMagicRate = Constants.CLIMBER_LEG_POSITION_RATE;
    }
    Robot.climberLeg.legTargetInches = myTarget;
  }

}
