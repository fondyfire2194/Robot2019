/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.DoFileTrajectory;
import frc.robot.commands.DoTeleopRobotOrient;
import frc.robot.commands.DoTeleopTrajectory;
import frc.robot.commands.DoTeleopPosition;
import frc.robot.commands.ResetEncoders;
import frc.robot.commands.ResetGyro;
import frc.robot.commands.DeleteAllPrefs;
import frc.robot.commands.Limelight.*;
import frc.robot.LimelightControlMode.*;
import frc.robot.LimeLight;
/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
  public Joystick driverController = new Joystick(RobotMap.OI_DRIVER_CONTROLLER);
  public Joystick co_driverController = new Joystick(RobotMap.OI_CO_DRIVER_CONTROLLER);
  
  public OI(){
    Timer.delay(.02);

    SmartDashboard.putData(new ResetEncoders());
    Timer.delay(.02);
    SmartDashboard.putData(new ResetGyro());
    Timer.delay(.02);
    SmartDashboard.putData("OrientRobot", new DoTeleopRobotOrient());
    Timer.delay(.02);
    SmartDashboard.putData("DriveToTarget", new DoTeleopPosition());

    
    Timer.delay(.02);
    SmartDashboard.putData("Tel Traj", new DoTeleopTrajectory());
    Timer.delay(.02);
    SmartDashboard.putData("File Traj", new DoFileTrajectory());
    Timer.delay(.02);
    SmartDashboard.putData("LEDS On", new SetLimelightLeds(LedMode.kforceOn) );
    SmartDashboard.putData("LEDS Off", new SetLimelightLeds(LedMode.kforceOff) );

    SmartDashboard.putData("Toggle View", new ToggleCamMode());
    SmartDashboard.putData("Toggle Stream", new ToggleStreamMode());
  }
}
