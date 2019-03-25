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
public class SetClimbValues extends InstantCommand {
  /**
   * Add your docs here.
   */
  private double myInches;
  private double myAngle;
  private double myFinalInches;
  private double myFinalAngle;
  
  public SetClimbValues(double touchinches, double touchangle, double finalinches, double finalangle) {
    super();
    myInches = touchinches;
    myAngle = touchangle;
    myFinalInches = finalinches;
    myFinalAngle = finalangle;
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    Robot.climberLeg.climbTouchInches = myInches;
    Robot.climberArm.climbTouchAngle = myAngle;
    Robot.climberLeg.climbFinalInches = myFinalInches;
    Robot.climberArm.climbFinalAngle = myFinalAngle;
  }

}
