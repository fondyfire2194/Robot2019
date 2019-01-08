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
  public static double DRIVE_ENCODER_COUNTS_PER_INCH = 433;
  public static final double DRIVE_ENCODER_CTS_PER_FT = DRIVE_ENCODER_COUNTS_PER_INCH * 12;
  public static final double MINIMUM_START_PCT = .1;
  public static final double IN_POSITION_BAND_FT = .5;
  public static final double MAX_ROBOT_FT_PER_SEC = 7.5;
  public static final double FT_PER_SEC_TO_ENC_CTS_PER_100MS = DRIVE_ENCODER_COUNTS_PER_INCH * 1.2;
  public static double MAX_ENC_CTS_PER_100MS = MAX_ROBOT_FT_PER_SEC * FT_PER_SEC_TO_ENC_CTS_PER_100MS;
  public static double FT_PER_SEC_TO_PCT_OUT = 1 / MAX_ROBOT_FT_PER_SEC;
  public final static double WHEELBASE_WIDTH = 2.1;


  private Constants() {

  }
}
