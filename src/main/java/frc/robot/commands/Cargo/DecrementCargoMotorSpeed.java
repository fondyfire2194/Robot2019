/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.Cargo;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.subsystems.GamePieceHandler;

/**
 * Add your docs here.
 */
public class DecrementCargoMotorSpeed extends InstantCommand {
  /**
   * Add your docs here.
   */
  public DecrementCargoMotorSpeed() {
    super();
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
  }

  // Called once when the command executes
  @Override
  protected void initialize() {
    double temp = GamePieceHandler.cargoMotor.getMotorOutputPercent();
    if (temp > 0) {
      temp -= .1;
      if (temp < 0)
        temp = 0;
    } else {
      temp += .1;
      if (temp > 0)
        temp = 0;
    }
    GamePieceHandler.cargoMotor.set(ControlMode.PercentOutput, temp);
  }

}
