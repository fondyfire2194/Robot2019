/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {
  /**
   * 
   * 
   * 
   * 
   */
  // Constants for drive
  public static double DRIVE_ENCODER_COUNTS_PER_INCH = 55;
  public static final double DRIVE_ENCODER_CTS_PER_FT = DRIVE_ENCODER_COUNTS_PER_INCH * 12;
  public static final double MINIMUM_START_PCT = 0.1;
  public static final double IN_POSITION_BAND_FT = .5;
  public static final double MAX_ROBOT_FT_PER_SEC = 7.5;
  public static final double FT_PER_SEC_TO_ENC_CTS_PER_100MS = DRIVE_ENCODER_COUNTS_PER_INCH * 1.2;
  public static double MAX_ENC_CTS_PER_100MS = MAX_ROBOT_FT_PER_SEC * FT_PER_SEC_TO_ENC_CTS_PER_100MS;
  public static double FT_PER_SEC_TO_PCT_OUT = 1 / MAX_ROBOT_FT_PER_SEC;
  public final static double WHEELBASE_WIDTH = 2.1;
  public final static double MAX_TARGET_AREA = 6.5;
  public final static double ANGLE_APPROACH_LIMIT = 5.;
  public final static double DRIVE_SIDE_STALL_DETECT = 20.;

  public static double ALL_LOWER_HATCH_INCHES = 0;
  public static double ROCKET_MID_HATCH_INCHES = ALL_LOWER_HATCH_INCHES + 28;
  public static double ROCKET_TOP_HATCH_INCHES = ROCKET_MID_HATCH_INCHES + 28;

  public static double SHIP_CARGO_INCHES = 13;
  public static double ROCKET_LOWER_CARGO_INCHES =  8;
  public static double ROCKET_MID_CARGO_INCHES =  ROCKET_LOWER_CARGO_INCHES +28;
  public static double ROCKET_TOP_CARGO_INCHES =  ROCKET_MID_CARGO_INCHES + 28;



  public static double ELEVATOR_POSITION_RATE = 30;// in per sec
  public static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 341.3;
  // (in/sec) * enc Counts/in = enc counts / sec then divide by 10 for 100ms
  public static double ELEVATOR_IN_PER_SEC_TO_ENC_CTS_PER_100MS = ELEVATOR_ENCODER_COUNTS_PER_INCH / 10;

  public static double ELEVATOR_MIN_HEIGHT = -.50;
  public static double ELEVATOR_MAX_HEIGHT = 89;
  public static double ELEVATOR_IN_POSITION_BAND = 3;

  public static boolean usePathWeaver = true;

  private Constants() {

  }
}
