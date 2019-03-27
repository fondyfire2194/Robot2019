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
public class IncrementClimberLeg extends InstantCommand {
  /**
   * Add your docs here.
   */
  double myIncrement;

  public IncrementClimberLeg(double increment) {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    myIncrement = increment;
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.climberLeg.legTargetInches+= myIncrement;
    if(Robot.climberLeg.legTargetInches < 0){
      Robot.climberLeg.legTargetInches = 0;
    }
    if(Robot.climberLeg.legTargetInches > 30){
      Robot.climberLeg.legTargetInches = 30;
    }


  }

}