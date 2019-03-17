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


  public double legHoldPositionDegrees;

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


  public void updateStatus() {

    SD.putN("ClimberLegPosition", (double) getLegEncoderPosition());
    SD.putN2("ClimberLegAmps", climberLeg.getOutputCurrent());
  }
}
