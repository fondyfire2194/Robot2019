/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

  public static final int AIR_COMPRESSOR = 0;
  public static final int POWER_DISTRIBUTION_PANEL = 1;
  public static final int DRIVETRAIN_LEFT_TALON_ONE = 3;
  public static final int DRIVETRAIN_LEFT_TALON_TWO = 4;
  
  public static final int DRIVETRAIN_RIGHT_TALON_ONE = 6;
  public static final int DRIVETRAIN_RIGHT_TALON_TWO = 7;

  public static final int CLIMBER_ARM = 8;
  public static final int CLIMBER_LEG = 9;
  public static final int CLIMBER_DRIVE = 11;

  public static final int CARGO_MOTOR = 10;
  public static final int ELEVATOR_MOTOR = 12;

  public static final int ELEVATOR_TRAVEL_SWITCH = 0;
  public static final int LEFT_PUSHER_SWITCH = 1;
  public static final int RIGHT_PUSHER_SWITCH = 2;

  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  public static final int OI_DRIVER_CONTROLLER = 0;
  public static final int OI_CO_DRIVER_CONTROLLER = 1;
  public static final int BUTTON_BOX = 2;
  public static final int OI_TEST_CONTROLLER = 3;
  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
}
