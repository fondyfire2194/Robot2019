/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.SD;
import frc.robot.Constants;
import frc.robot.commands.Climber.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class ClimberArm extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX climberArm = null;

  public double armTargetDegrees;
  public double lastHoldDegrees;

  public ClimberArm() {
    climberArm = new TalonSRX(RobotMap.CLIMBER_ARM);

    climberArm.configFactoryDefault();

    climberArm.setInverted(false);

    climberArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    climberArm.setSensorPhase(true);

    climberArm.setNeutralMode(NeutralMode.Brake);
    climberArm.configVoltageCompSaturation(12, 0);
    climberArm.enableVoltageCompensation(true);
    climberArm.set(ControlMode.PercentOutput, 0);

    climberArm.setSelectedSensorPosition(0, 0, 0);
    configVelMode();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    // setDefaultCommand(new RunClimberArmFromGamepad(false));
    setDefaultCommand(new ClimberArmMotionMagic());
  }

  public void configVelMode() {
    climberArm.config_kF(1, 2., 0);
    climberArm.config_kP(1, .4, 0);
    climberArm.config_kI(1, 0, 0);
    climberArm.config_kD(1, 0, 0);

    climberArm.configNominalOutputForward(0, 0);
    climberArm.configNominalOutputReverse(0, 0);
    climberArm.configPeakOutputForward(1, 0);
    climberArm.configPeakOutputReverse(-1, 0);

  }

  public int getArmEncoderPosition() {
    return climberArm.getSelectedSensorPosition(0);
  }

  public double getArmDegrees() {
    return getArmEncoderPosition() / Constants.CLIMBER_ARM_COUNTS_PER_DEGREE;

  }

  public void resetArmPosition() {
    climberArm.setSelectedSensorPosition(0, 0, 0);
  }

  public boolean armInPosition() {
    return Math.abs(armTargetDegrees - getArmDegrees()) < 5;
  }

  public void climberArmConfigCurrents(int peak, int time, int continuous) {
    climberArm.configPeakCurrentLimit(peak);
    climberArm.configPeakCurrentDuration(time);
    climberArm.configContinuousCurrentLimit(continuous);
  }

  public double getArmSpeed() {
    return climberArm.getSelectedSensorVelocity(0) * 10 / Constants.CLIMBER_ARM_COUNTS_PER_DEGREE;

  }

  public void climberArmOut(double speed) {
    climberArm.set(ControlMode.PercentOutput, speed);
  }

  public void armMagicMotion(double distance, double speedDPS) {
    /**
     * arm motor 775 Pro with 343:1 gear reduction and a 4096 count encoder 53
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

    armTargetDegrees = distance;

    /*
     * set acceleration and cruise velocity - see documentation accel is in units of
     * enc cts per 100ms per second so to accelerate in 1/2 second, use velocity x 2
     */

    int cruiseVelocity = (int) (speedDPS * Constants.ARM_DEG_PER_SEC_TO_ENC_CTS_PER_100MS);

    int acceleration = cruiseVelocity * 2;

    climberArm.configMotionCruiseVelocity(cruiseVelocity, 0);
    climberArm.configMotionAcceleration(acceleration, 0);
    climberArm.set(ControlMode.MotionMagic, distance * Constants.CLIMBER_ARM_COUNTS_PER_DEGREE);
  }

  public void updateStatus() {
    SD.putN("ClimberArmPosition", (double) getArmEncoderPosition());
    SD.putN1("ClimberArmDegrees", getArmDegrees());
    SD.putN2("ClimberArmAmps", climberArm.getOutputCurrent());
    SD.putN1("ClimberArmENCPer100MS", climberArm.getSelectedSensorVelocity(0));
    SD.putN1("ClimberArmTarget", armTargetDegrees);
  }
}
