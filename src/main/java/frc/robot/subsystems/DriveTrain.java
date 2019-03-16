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
import frc.robot.MovingAverage;
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
  private int lastLeftEncoderValue;
  private int lastRightEncoderValue;
  private MovingAverage movingAverage;

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public DriveTrain() {
    leftTalonOne = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_TALON_ONE);
    leftTalonTwo = new TalonSRX(RobotMap.DRIVETRAIN_LEFT_TALON_TWO);

    rightTalonOne = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_TALON_ONE);
    rightTalonTwo = new TalonSRX(RobotMap.DRIVETRAIN_RIGHT_TALON_TWO);

    leftTalonOne.configFactoryDefault();
    leftTalonTwo.configFactoryDefault();
    rightTalonOne.configFactoryDefault();
    rightTalonTwo.configFactoryDefault();

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
    configVoltageCompSaturation(true);

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
    movingAverage = new MovingAverage(10);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());

    setDefaultCommand(new JoystickArcadeDrive());
    // setDefaultCommand(new JoystickArcadeDriveVision());
  }

  public void leftDriveOut(double speed) {
    if (useVelocityLoop || Robot.trajectoryRunning || Robot.orientRunning) {
      leftTalonOne.selectProfileSlot(0, 0);
      leftTalonOne.set(ControlMode.Velocity, speed * Constants.MAX_ENC_CTS_PER_100MS);
    } else {
      leftTalonOne.set(ControlMode.PercentOutput, speed);
    }
  }

  public void rightDriveOut(double speed) {
    if (useVelocityLoop || Robot.trajectoryRunning || Robot.orientRunning) {
      rightTalonOne.selectProfileSlot(0, 0);
      rightTalonOne.set(ControlMode.Velocity, speed * Constants.MAX_ENC_CTS_PER_100MS);
    } else {
      rightTalonOne.set(ControlMode.PercentOutput, speed);
    }
  }

  public void arcadeDrive(double throttleValue, double turnValue) {
    leftDriveOut(throttleValue + turnValue);
    rightDriveOut(throttleValue - turnValue);
  }

  public void enableBothSidesCurrentLimit(boolean enable) {
    leftTalonOne.enableCurrentLimit(enable);
    leftTalonTwo.enableCurrentLimit(enable);
    rightTalonOne.enableCurrentLimit(enable);
    rightTalonTwo.enableCurrentLimit(enable);
  }

  public void configVoltageCompSaturation(boolean on) {
    leftTalonOne.configVoltageCompSaturation(12, 0);
    leftTalonOne.enableVoltageCompensation(on);
    leftTalonTwo.configVoltageCompSaturation(12, 0);
    leftTalonTwo.enableVoltageCompensation(on);
    rightTalonOne.configVoltageCompSaturation(12, 0);
    rightTalonOne.enableVoltageCompensation(on);
    rightTalonTwo.configVoltageCompSaturation(12, 0);
    rightTalonTwo.enableVoltageCompensation(on);
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

  public double getDriverSlider() {
    // want to change range from min pref to max pref from incoming 1 to -1
    // subtracting it from 1 makes range 0 t0 2
    // divide by 2 range is 0 to 1
    double temp = (1 - Robot.m_oi.driverController.getRawAxis(3)) / 2;
    double range = Math.abs(Pref.getPref("JSTwistMaxKp") - Pref.getPref("JSTwistMinKp"));
    return Pref.getPref("JSTwistMinKp") + range * temp;
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

  public double getGyroRoll() {
    return imu.getPitch();
  }

  public double getGyroPitch() {
    return imu.getRoll();
  }

  public double getFilteredGyroPitch() {
    movingAverage.add(getGyroPitch());
    return movingAverage.getAverage();
  }

  public boolean isRotating() {
    return imu.isRotating();
  }

  public boolean isMoving() {
    return imu.isMoving();
  }

  public double getXAccel() {
    return imu.getWorldLinearAccelX();
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
    return rightTalonOne.getOutputCurrent() > Pref.getPref("DriveStall");
  }

  public int getLeftEncoderChange() {
    int temp = getLeftEncoderCount() - lastLeftEncoderValue;
    lastLeftEncoderValue = getLeftEncoderCount();
    return temp;
  }

  public int getRightEncoderChange() {
    int temp = getRightEncoderCount() - lastRightEncoderValue;
    lastRightEncoderValue = getRightEncoderCount();
    return temp;
  }

  public boolean leftEncoderNoChange() {
    return Math.abs(getLeftEncoderChange()) < 2;
  }

  public boolean rightEncoderNoChange() {
    return Math.abs(getRightEncoderChange()) < 2;
  }

  public void updateStatus() {

    SmartDashboard.putNumber("GyOff", gyroOffset);
    SmartDashboard.putNumber("DrvStrtAng", driveStraightAngle);
    SmartDashboard.putNumber("Right ft per s", getRightFeetPerSecond());
    SmartDashboard.putNumber("Left ft per s", getLeftFeetPerSecond());
    SD.putN2("LeftFeet", getLeftFeet());
    SD.putN2("RightFeet", getRightFeet());
    SD.putN1("GyroYaw", getGyroYaw());
    SD.putN1("GyroPitch", getGyroPitch());
    SD.putN1("GyroRoll", getGyroRoll());
    useVelocityLoop = SmartDashboard.getBoolean("DriveCloseLoop", false);
    SmartDashboard.putNumber("Slider", getDriverSlider());
    SmartDashboard.putNumber("XAccel", getXAccel());
    SmartDashboard.putNumber("SSV", leftTalonOne.getSelectedSensorVelocity(0));
    SmartDashboard.putNumber("SSE", leftTalonOne.getClosedLoopError(0));
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
