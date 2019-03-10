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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class Climber extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX climberArm = null;
  public TalonSRX climberLeg = null;

  public Climber(){
    climberArm = new TalonSRX(RobotMap.CLIMBER_ARM);
    climberLeg = new TalonSRX(RobotMap.CLIMBER_LEG);
    climberArm.configFactoryDefault();
    climberLeg.configFactoryDefault();
    climberArm.setInverted(false);
    climberLeg.setInverted(false);
    climberArm.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
    climberArm.setSensorPhase(false);
    climberArm.setNeutralMode(NeutralMode.Brake);
		climberArm.configVoltageCompSaturation(12, 0);
		climberArm.enableVoltageCompensation(true);
    climberLeg.setNeutralMode(NeutralMode.Brake);
		climberLeg.configVoltageCompSaturation(12, 0);
    climberLeg.enableVoltageCompensation(true);
    climberArm.set(ControlMode.PercentOutput,0);
    climberLeg.set(ControlMode.PercentOutput,0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public int getClimberArmEncoderPosition() {
		return climberArm.getSelectedSensorPosition(0);
  }
  
  public void runClimberLeg(double speed){
    climberLeg.set(ControlMode.PercentOutput,speed);
  }

  public void runClimberArm(double speed){
    climberArm.set(ControlMode.PercentOutput,speed);
  }

  public void updateStatus(){
    SD.putN("CliberArmPosition",(double)getClimberArmEncoderPosition());
  }
}
