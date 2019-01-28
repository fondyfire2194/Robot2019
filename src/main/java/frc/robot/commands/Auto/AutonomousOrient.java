/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Auto;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.RobotOrient;

public class AutonomousOrient extends CommandGroup {
  /**
   * Add your docs here.
   */

  private double mySpeed;
	private double myAngle;
	private double myTimeout;
	
  private boolean myAccuracy;
  


  public AutonomousOrient(double angle, double speed, boolean accuracy, double timeout, int step) {

    mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
    myAccuracy = accuracy;
    
    
    addSequential(new RobotOrient(mySpeed, myAngle, myAccuracy, myTimeout));
    addSequential(new SetAutoCommandDone(step));

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
  }
}
