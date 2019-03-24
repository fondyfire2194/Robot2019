/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.SD;
import frc.robot.Constants;
import frc.robot.commands.Climber.*;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * enc cts per inch = 700 max inches per sec = 8 max enc cts per 100 ms = 700 *
 * 8 / 10 = 560 kf = 1023 / 560 = 1.8 say 1.4 if error = 1 in / sec then error =
 * 70 enc cts per 100 ms p gain = corr * 1023 / error for 5% correction P = 5 *
 * 1023/70 = 7
 */
public class ClimberLeg extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX climberLeg = null;
  public static DigitalInput legSwitch;
  public double motionMagicRate = Constants.CLIMBER_LEG_POSITION_RATE;
  public double legTargetInches;
  public double lastHoldInches;
  public boolean legOnSwitch;
  private int switchCounter;
  private boolean switchWasSeen;

  public ClimberLeg() {

    climberLeg = new TalonSRX(RobotMap.CLIMBER_LEG);
    climberLeg.configFactoryDefault();
    climberLeg.setInverted(true);

    climberLeg.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    climberLeg.setSensorPhase(true);
    climberLeg.setNeutralMode(NeutralMode.Brake);
    climberLeg.configVoltageCompSaturation(12, 0);
    climberLeg.enableVoltageCompensation(true);
    resetLegPosition();

    climberLeg.configOpenloopRamp(0, 0);
    climberLeg.configClosedloopRamp(0, 0);
    climberLeg.configPeakOutputForward(1, 0);
    climberLeg.configPeakOutputReverse(-1, 0);
    climberLeg.configNominalOutputForward(0, 0);
    climberLeg.configNominalOutputReverse(0, 0);
    // motion magic
    climberLeg.config_kF(0, 1.4, 0);
    climberLeg.config_kP(0, 4, 0);
    climberLeg.config_kI(0, 0, 0);
    climberLeg.config_kD(0, 0, 0);
    // velocity
    climberLeg.config_kF(1, 1.4, 0);
    climberLeg.config_kP(1, 1, 0);
    climberLeg.config_kI(1, 0, 0);
    climberLeg.config_kD(1, 0, 0);

    legSwitch = new DigitalInput(RobotMap.CLIMBER_LEG_TRAVEL_SWITCH);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new RunClimberLegFromGamepad(false));
    setDefaultCommand(new ClimberLegMotionMagic());
  }

  public int getLegEncoderPosition() {
    return climberLeg.getSelectedSensorPosition(0);
  }

  public double getLegInches() {
    return getLegEncoderPosition() / Constants.CLIMBER_LEG_COUNTS_PER_INCH;
  }

  public void resetLegPosition() {
    climberLeg.setSelectedSensorPosition(0, 0, 0);
  }

  public void runClimberLeg(double speed) {
    climberLeg.set(ControlMode.PercentOutput, speed);
  }

  public void climberLegConfigCurrents(int peak, int time, int continuous) {
    climberLeg.configPeakCurrentLimit(peak);
    climberLeg.configPeakCurrentDuration(time);
    climberLeg.configContinuousCurrentLimit(continuous);
  }

  public double getLegCurrent() {
    return climberLeg.getOutputCurrent();
  }

  public double getLegPercentOut(){
    return climberLeg.getMotorOutputPercent();
  }

  public double getLegInPerSec() {

    return (climberLeg.getSelectedSensorVelocity(0) * 10) / Constants.CLIMBER_LEG_COUNTS_PER_INCH;
  }

  public void climberLegOut(double speed, boolean useVelocityLoop) {
    SD.putN3("LegSpeed", speed);
    if (useVelocityLoop) {
      climberLeg.selectProfileSlot(1, 0);
      climberLeg.set(ControlMode.Velocity, speed * Constants.MAX_LEG_ENC_CTS_PER_100MS);
    } else {
      climberLeg.set(ControlMode.PercentOutput, speed);
    }
  }

  public void legMagicMotion(double distance, double speed) {
    /**
     * leg motor 775 Pro with 343:1 gear reduction and a 4096 count encoder 53
     * counts per degree
     * 
     * 18000 / 343 = 52 rpm encoder = .8 rpsec = .08 rper 100ms
     * 
     * 
     *
     * = 320 counts / 100 ms measured encoder rate at 100% was 250
     *
     * Use measured rate not theoretical so 100% Kf would be 1023/250 = 4
     * 
     * For error of 1 degree, to add another 2% of motor output. p-gain .02 x 1023 /
     * (52) = .4
     * 
     * start P-gain = .06
     * 
     * 
     */

    legTargetInches = distance;

    /*
     * set acceleration and cruise velocity - see documentation accel is in units of
     * enc cts per 100ms per second so to accelerate in 1/2 second, use velocity x 2
     */

    int cruiseVelocity = (int) (speed * Constants.LEG_INCHES_PER_SEC_TO_ENC_CTS_PER_100MS);

    int acceleration = cruiseVelocity * 2;

    climberLeg.configMotionCruiseVelocity(cruiseVelocity, 0);
    climberLeg.configMotionAcceleration(acceleration, 0);
    climberLeg.set(ControlMode.MotionMagic, distance * Constants.CLIMBER_LEG_COUNTS_PER_INCH);
  }

  public void updateStatus() {
    legOnSwitch = !legSwitch.get();
    
    if (legOnSwitch)
    switchCounter++;
  else
    switchCounter = 0;

  if (switchCounter > 10 && !switchWasSeen) {
    resetLegPosition();
    legTargetInches = getLegInches();
    switchWasSeen = true;
  }
  if (switchWasSeen)
    switchWasSeen = legOnSwitch;
    
    SD.putN("ClimberLegPosition", (double) getLegEncoderPosition());
    SD.putN1("ClimberLegInches", getLegInches());
    SD.putN1("ClimberLegENCPer100MS", climberLeg.getSelectedSensorVelocity(0));
    SD.putN1("ClimberLegInchesPerSec", getLegInPerSec());
    SD.putN1("ClimberLegTarget", legTargetInches);
    SD.putN2("ClimberLegAmps", getLegCurrent());
    SmartDashboard.putBoolean("ClimberLegSwitch", legOnSwitch);

  }
}
