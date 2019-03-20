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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class ClimberLeg extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.


  public TalonSRX climberLeg = null;


  public double legTargetInches;
  public double lastHoldInches;

  public ClimberLeg() {

    climberLeg = new TalonSRX(RobotMap.CLIMBER_LEG);
    climberLeg.configFactoryDefault();
    climberLeg.setInverted(false);


    climberLeg.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    climberLeg.setSensorPhase(true);
    climberLeg.setNeutralMode(NeutralMode.Brake);
    climberLeg.configVoltageCompSaturation(12, 0);
    climberLeg.enableVoltageCompensation(true);


  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
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


  public void climberLegOut(double speed, boolean useVelocityLoop) {
    if (useVelocityLoop) {
      climberLeg.selectProfileSlot(0, 0);
      climberLeg.set(ControlMode.Velocity, speed * Constants.MAX_ENC_CTS_PER_100MS);
    } else {
      climberLeg.set(ControlMode.PercentOutput, speed);
    }
  }

  public void legMagicMotion(double distance, double speedDPS) {
    /**
     * arm motor 775 Pro with 343:1 gear reduction and a 4096 count encoder
     *  53 counts per degree
     * 
     * 18000 / 343 = 52 rpm encoder = .8 rpsec = .08 rper 100ms 
     * 
     * 
     *
     * = 320 counts / 100 ms measured encoder rate at 100% was 250
     *
     * Use measured rate not theoretical so 100% Kf would be 1023/250 = 4 
     * 
     * For error of 1 degree, to add another 2% of motor output. p-gain .02 x 1023 / (52) =
     * .4
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

    int cruiseVelocity = (int) (speedDPS * Constants.ARM_DEG_PER_SEC_TO_ENC_CTS_PER_100MS);

    int acceleration = cruiseVelocity * 2;

    climberLeg.configMotionCruiseVelocity(cruiseVelocity, 0);
    climberLeg.configMotionAcceleration(acceleration, 0);
    climberLeg.set(ControlMode.MotionMagic, distance * Constants.CLIMBER_LEG_COUNTS_PER_INCH);
  }




  public void updateStatus() {

    SD.putN("ClimberLegPosition", (double) getLegEncoderPosition());
    SD.putN1("ClimberLegInches",getLegInches());
    SD.putN1("ClimberLegENCPer100MS",climberLeg.getSelectedSensorVelocity(0));
    SD.putN1("ClimberLegTarget", legTargetInches);
    SD.putN2("ClimberLegAmps", climberLeg.getOutputCurrent());

  }
}
