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

  public double armHoldPositionDegrees;

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

    		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		climberArm.config_kF(0, 0, 0);
		climberArm.config_kP(0, 0, 0);
		climberArm.config_kI(0, 0, 0);
		climberArm.config_kD(0, 0, 0); 
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
    setDefaultCommand(new RunClimberArmFromGamepad());
    // setDefaultCommand(new ClimberArmPosition());
  }

  public int getArmEncoderPosition() {
    return climberArm.getSelectedSensorPosition(0);
  }

  public double getArmDegrees() {
    return getArmEncoderPosition() / Constants.CLIMBER_ARM_COUNTS_PER_DEGREE;

  }

  public void resetArmPosition() {
    climberArm.setSelectedSensorPosition(0, 0, 0);
    armHoldPositionDegrees = getArmDegrees();
  }

  public boolean armInPosition(){
    return Math.abs(99 - getArmDegrees() )<5;
  }

  public void climberArmConfigCurrents(int peak, int time, int continuous) {
    climberArm.configPeakCurrentLimit(peak);
    climberArm.configPeakCurrentDuration(time);
    climberArm.configContinuousCurrentLimit(continuous);
  }


  public void runClimberArm(double speed) {
    climberArm.set(ControlMode.PercentOutput, speed);
  }

  public void climberArmOut(double speed, boolean useVelocityLoop) {
    if (useVelocityLoop) {
      climberArm.selectProfileSlot(0, 0);
      climberArm.set(ControlMode.Velocity, speed * Constants.MAX_ARM_ENC_CTS_PER_100MS);
    } else {
      climberArm.set(ControlMode.PercentOutput, speed);
    }
  }



  public void updateStatus() {
    SD.putN("CliberArmPosition", (double) getArmEncoderPosition());
    SD.putN2("ClimberArmAmps", climberArm.getOutputCurrent());
  }
}
