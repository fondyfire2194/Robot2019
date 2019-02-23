/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.Teleop.JoystickArcadeDrive;
import frc.robot.ReverseDistanceFollower;
import frc.robot.SD;
import frc.robot.Constants;
import frc.robot.Pref;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.*;
import frc.robot.commands.Teleop.JoystickArcadeDriveVision;
import frc.robot.AutoChoosers;

/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  public TalonSRX leftTalonOne = null;
  public TalonSRX leftTalonTwo = null;
  public TalonSRX rightTalonOne = null;
  public TalonSRX rightTalonTwo = null;
  public static AHRS imu;

  public DistanceFollower leftDf = new DistanceFollower();
  public DistanceFollower rightDf = new DistanceFollower();
  public ReverseDistanceFollower revLeftDf = new ReverseDistanceFollower();
  public ReverseDistanceFollower revRightDf = new ReverseDistanceFollower();
  public boolean leftSideStopped;
  public boolean rightSideStopped;
  public double leftPositionTargetFt;
  public double rightPositionTargetFt;
  public boolean useGyroComp;
  public double driveStraightAngle = 0;
  public static double gyroOffset = 0;
  public static boolean useVelocityLoop;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DriveTrain() {
    leftTalonOne = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_TALON_ONE);
    leftTalonTwo = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_TALON_TWO);

    rightTalonOne = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_TALON_ONE);
    rightTalonTwo = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_TALON_TWO);

    rightTalonOne.setInverted(true);
    rightTalonTwo.setInverted(true);

    leftTalonTwo.set(ControlMode.Follower, RobotMap.DRIVETRAIN_LEFT_TALON_ONE);
    leftTalonOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    leftTalonOne.setSensorPhase(false);
    setLeftSideDriveBrakeOn(true);
    rightTalonTwo.set(ControlMode.Follower, RobotMap.DRIVETRAIN_RIGHT_TALON_ONE);
    rightTalonOne.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    rightTalonOne.setSensorPhase(false);
    setRightSideDriveBrakeOn(true);

    leftTalonOne.selectProfileSlot(0, 0);
    leftTalonOne.config_kF(0, Pref.getPref("DriveVelKf"), 0);
    leftTalonOne.config_kP(0, Pref.getPref("DriveVelKp"), 0);
    leftTalonOne.config_kI(0, Pref.getPref("DriveVelKi"), 0);
    leftTalonOne.config_kD(0, Pref.getPref("DriveVelKd"), 0);

    rightTalonOne.selectProfileSlot(0, 0);
    rightTalonOne.config_kF(0, Pref.getPref("DriveVelKf"), 0);
    rightTalonOne.config_kP(0, Pref.getPref("DriveVelKp"), 0);
    rightTalonOne.config_kI(0, Pref.getPref("DriveVelKi"), 0);
    rightTalonOne.config_kD(0, Pref.getPref("DriveVelKd"), 0);

    try {
      // imu = new AHRS(I2C.Port.kOnboard);

      imu = new AHRS(SPI.Port.kMXP);
      // imu = new AHRS(SerialPort.Port.kUSB1);

      imu.setPIDSourceType(PIDSourceType.kDisplacement);

    } catch (Exception ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new JoystickArcadeDrive());
    // setDefaultCommand(new JoystickArcadeDriveVision());
  }

  public void leftDriveOut(double speed) {
    if (!useVelocityLoop) {
      leftTalonOne.set(ControlMode.PercentOutput, speed);
    } else {
      leftTalonOne.selectProfileSlot(0, 0);
      leftTalonOne.set(ControlMode.Velocity, speed * Constants.MAX_ENC_CTS_PER_100MS);
    }
  }

  public void rightDriveOut(double speed) {
    if (!useVelocityLoop) {
      rightTalonOne.set(ControlMode.PercentOutput, speed);
    } else {
      rightTalonOne.selectProfileSlot(0, 0);
      rightTalonOne.set(ControlMode.Velocity, speed * Constants.MAX_ENC_CTS_PER_100MS);
    }
  }

  public void arcadeDrive(double throttleValue, double turnValue) {
    SmartDashboard.putNumber("LDAout", throttleValue);
    leftDriveOut(throttleValue + turnValue);
    rightDriveOut(throttleValue - turnValue);
  }

  public void setLeftSideDriveBrakeOn(boolean on) {
    if (on) {
      leftTalonOne.setNeutralMode(NeutralMode.Brake);
      leftTalonTwo.setNeutralMode(NeutralMode.Brake);
    } else {
      leftTalonOne.setNeutralMode(NeutralMode.Coast);
      leftTalonTwo.setNeutralMode(NeutralMode.Coast);
    }
  }

  public void setRightSideDriveBrakeOn(boolean on) {
    if (on) {
      rightTalonOne.setNeutralMode(NeutralMode.Brake);
      rightTalonTwo.setNeutralMode(NeutralMode.Brake);
    } else {
      rightTalonOne.setNeutralMode(NeutralMode.Coast);
      rightTalonTwo.setNeutralMode(NeutralMode.Coast);
    }
  }

  public int getLeftEncoderCount() {
    return leftTalonOne.getSelectedSensorPosition(0);
  }

  public int getRightEncoderCount() {
    return rightTalonOne.getSelectedSensorPosition(0);
  }

  public double getLeftFeet() {
    return ((double) getLeftEncoderCount()) / Constants.DRIVE_ENCODER_CTS_PER_FT;
  }

  public double getRightFeet() {
    return ((double) getRightEncoderCount()) / Constants.DRIVE_ENCODER_CTS_PER_FT;
  }

  public double getLeftFeetPerSecond() {
    return ((leftTalonOne.getSelectedSensorVelocity(0)) / Constants.FT_PER_SEC_TO_ENC_CTS_PER_100MS);
  }

  public double getRightFeetPerSecond() {
    return ((rightTalonOne.getSelectedSensorVelocity(0)) / Constants.FT_PER_SEC_TO_ENC_CTS_PER_100MS);
  }

  public boolean inPosition() {
    return Math.abs(leftPositionTargetFt - getLeftFeet()) < Constants.IN_POSITION_BAND_FT;
  }

  public double getPositionError() {
    return Robot.positionTargetFt - getLeftFeet();
  }

  public void resetEncoders() {
    rightTalonOne.setSelectedSensorPosition(0, 0, 0);
    leftTalonOne.setSelectedSensorPosition(0, 0, 0);
  }

  public double getGyroYaw() {
    return Pathfinder.boundHalfDegrees(imu.getYaw() + gyroOffset);
  }

  public boolean isRotating() {
    return imu.isRotating();
  }

  public boolean isMoving() {
    return imu.isMoving();
  }

  public double getGyroError() {
    return imu.getYaw() - driveStraightAngle;
  }

  public void resetGyro() {
    imu.reset();
  }

  public double getLeftCommand() {
    return leftTalonOne.getMotorOutputPercent();
  }

  public double getRightCommand() {
    return rightTalonOne.getMotorOutputPercent();
  }

  public double getCurrentComp() {
    if (useGyroComp)
      return -(getGyroError() * Pref.getPref("DriveStraightKp"));
    else
      return 0;
  }

  public boolean getLeftSideStalled() {
    return leftTalonOne.getOutputCurrent() > Pref.getPref("DriveStall");
  }

  public boolean getRightSideStalled() {
    return rightTalonOne.getOutputCurrent() >  Pref.getPref("DriveStall");
  }

  public void updateStatus() {

    SmartDashboard.putNumber("Right ft per s", getRightFeetPerSecond());
    SmartDashboard.putNumber("Left ft per s", getLeftFeetPerSecond());
    SD.putN2("LeftFeet", getLeftFeet());
    SD.putN2("RightFeet", getRightFeet());
    SD.putN1("GyroYaw", getGyroYaw());
    useVelocityLoop = SmartDashboard.getBoolean("DriveCloseLoop", false);

    if (AutoChoosers.debugChooser.getSelected() == 2) {

      SD.putN2("LeftCmd", getLeftCommand());
      SD.putN2("RightCmd", getRightCommand());
      SD.putN2("LeftA Amps", leftTalonOne.getOutputCurrent());
      SD.putN2("LeftB Amps", leftTalonTwo.getOutputCurrent());

      SD.putN2("RightA Amps", rightTalonOne.getOutputCurrent());
      SD.putN2("RightB Amps", rightTalonTwo.getOutputCurrent());
      SmartDashboard.putNumber("LeftEncoder", getLeftEncoderCount());
      SmartDashboard.putNumber("RightEncoder", getRightEncoderCount());
      SmartDashboard.putNumber("VelKf", 1 / Constants.MAX_ENC_CTS_PER_100MS);
SmartDashboard.putBoolean("LSStall", getLeftSideStalled());
SmartDashboard.putBoolean("TSStall", getRightSideStalled());
    }

  }
}
