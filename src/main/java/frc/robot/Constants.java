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
  public static double DRIVE_LEFT_ENCODER_COUNTS_PER_INCH = 55;
  public static double DRIVE_RIGHT_ENCODER_COUNTS_PER_INCH = 55;
  public static double DRIVE_ENCODER_COUNTS_PER_INCH = (DRIVE_LEFT_ENCODER_COUNTS_PER_INCH + DRIVE_RIGHT_ENCODER_COUNTS_PER_INCH)/2;
  public static final double DRIVE_LEFT_ENCODER_CTS_PER_FT = DRIVE_LEFT_ENCODER_COUNTS_PER_INCH * 12;
  public static final double DRIVE_RIGHT_ENCODER_CTS_PER_FT = DRIVE_RIGHT_ENCODER_COUNTS_PER_INCH * 12;
  public static final double MINIMUM_START_PCT = 0.2;
  public static final double IN_POSITION_BAND_FT = .5;
  public static final double MAX_ROBOT_FT_PER_SEC = 10.;
  public static final double FT_PER_SEC_TO_ENC_CTS_PER_100MS = DRIVE_ENCODER_COUNTS_PER_INCH * 1.2;
  public static double MAX_ENC_CTS_PER_100MS = MAX_ROBOT_FT_PER_SEC * FT_PER_SEC_TO_ENC_CTS_PER_100MS;
  public static double FT_PER_SEC_TO_PCT_OUT = 1 / MAX_ROBOT_FT_PER_SEC;
  public final static double WHEELBASE_WIDTH = 1.88;
  public final static double MAX_TARGET_AREA = 10.;
  public final static double MAX_AREA_DISTANCE = 2.0;
  public final static double ANGLE_APPROACH_LIMIT = 5.;
  public final static double DRIVE_SIDE_STALL_DETECT = 50.;
  public final static double CAMERA_TO_FRONT_OF_BUMPER = 1.0;

  public final static double ORIENT_RATE = .35;
  public final static double POSITION_RATE = 7.;
  public static double ALL_LOWER_HATCH_INCHES = 0.;
  public static double ROCKET_MID_HATCH_INCHES = ALL_LOWER_HATCH_INCHES + 28;
  public static double ROCKET_TOP_HATCH_INCHES = ROCKET_MID_HATCH_INCHES + 28;

  public static double SHIP_CARGO_INCHES = 36;
  public static double ROCKET_LOWER_CARGO_INCHES = 24;
  public static double ROCKET_MID_CARGO_INCHES = ROCKET_LOWER_CARGO_INCHES + 28;
  public static double ROCKET_TOP_CARGO_INCHES = ROCKET_MID_CARGO_INCHES + 28;
// ********************************************************************
  public static double ELEVATOR_POSITION_RATE = 40;// in per sec
  public static double ELEVATOR_ENCODER_COUNTS_PER_INCH = 341.3;
  // (in/sec) * enc Counts/in = enc counts / sec then divide by 10 for 100ms
  public static double ELEVATOR_IN_PER_SEC_TO_ENC_CTS_PER_100MS = ELEVATOR_ENCODER_COUNTS_PER_INCH / 10;

  public static double ELEVATOR_MIN_HEIGHT = -.50;
  public static double ELEVATOR_MAX_HEIGHT = 69;
  public static double ELEVATOR_IN_POSITION_BAND = 3;

  public static double ELEVATOR_CARGO_LOAD = 20;
//**************************************************************** */
  public static double CLIMBER_LEG_COUNTS_PER_INCH = 700;
  public static double MAX_LEG_INCHES_PER_SEC = 8;
  public static double MAX_LEG_ENC_CTS_PER_100MS = MAX_LEG_INCHES_PER_SEC / 10 * CLIMBER_LEG_COUNTS_PER_INCH;//600
  public static double LEG_INCHES_PER_SEC_TO_ENC_CTS_PER_100MS = CLIMBER_LEG_COUNTS_PER_INCH / 10;
  public static double CLIMBER_LEG_POSITION_RATE = 4;//IPS
//****************************************************************** */
  public static double CLIMBER_ARM_COUNTS_PER_DEGREE = 53.;//
  public static double CLIMBER_ARM_LENGTH_INCHES = 18.;
  public static double MAX_ARM_DEG_PER_SEC = 50;
  public static double MAX_ARM_ENC_CTS_PER_100MS = MAX_ARM_DEG_PER_SEC / 10 * CLIMBER_ARM_COUNTS_PER_DEGREE;
  public static double ARM_DEG_PER_SEC_TO_ENC_CTS_PER_100MS = CLIMBER_ARM_COUNTS_PER_DEGREE / 10;
  public static double LEVEL_2_START_ANGLE = 35;
  public static double LEVEL_3_START_ANGLE = 75;
  public static double CLIMBER_ARM_CLIMB_RATE = 5;
  public static double CLIMBER_ARM_POSITION_RATE = 10;

  public static double CLIMBER_ARM_LEG_RATIO = .322;
 
   public static double CLIMBER_LEG_START_POSITION = 1.5;

  public static double CLIMB_TARGET_ANGLE = 5;

  public static double VISION_START_FEET = 10.0;
  public static double VISION_END_FEET = 4.0;

  public static double USND_CORRECT_BAND = 2.0;

  public static boolean usePathWeaver = true;

  private Constants() {

  }
}
