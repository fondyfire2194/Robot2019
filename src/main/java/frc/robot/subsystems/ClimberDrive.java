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
public class ClimberDrive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public TalonSRX climberDrive = null;


  public ClimberDrive() {

    climberDrive = new TalonSRX(RobotMap.CLIMBER_DRIVE);
    climberDrive.configFactoryDefault();
    climberDrive.setInverted(false);

    climberDrive.setNeutralMode(NeutralMode.Brake);
    climberDrive.configVoltageCompSaturation(12, 0);
    climberDrive.enableVoltageCompensation(true);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  public void climberDriveConfigCurrents(int peak, int time, int continuous) {
    climberDrive.configPeakCurrentLimit(peak);
    climberDrive.configPeakCurrentDuration(time);
    climberDrive.configContinuousCurrentLimit(continuous);
  }


  public void runClimberDrive(double speed) {
    climberDrive.set(ControlMode.PercentOutput, speed);
  }

  public void climberDriveOut(double speed) {
      climberDrive.set(ControlMode.PercentOutput, speed);
  }

  public void updateStatus() {

    SD.putN2("ClimberDriveAmps", climberDrive.getOutputCurrent());
  }
}
