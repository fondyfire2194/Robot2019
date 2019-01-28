package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.SD;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GamePieceHandler extends Subsystem {
	public static TalonSRX cargoMotor;
	public DoubleSolenoid hatchCoverGripper;
	public DoubleSolenoid hatchCoverExtend;

public GamePieceHandler(){

   cargoMotor= new TalonSRX(RobotMap.CARGO_MOTOR);

   hatchCoverGripper = new DoubleSolenoid(1,0);
   hatchCoverGripper.set(DoubleSolenoid.Value.kForward);

   hatchCoverExtend = new DoubleSolenoid(2,3);
   hatchCoverExtend.set(DoubleSolenoid.Value.kReverse);

	}

	// Put methods for controlling this subsystem
	// here. Call these from Commands.

	public void initDefaultCommand() {
		

	}

	public void stopCargoMotor(){
	    cargoMotor.set(ControlMode.PercentOutput, 0);	
	}
	
	public void pickUpCargo(double speed) {
		cargoMotor.set(ControlMode.PercentOutput, speed);	
	}

	public void deliverCargo(double speed) {
		cargoMotor.set(ControlMode.PercentOutput, -speed);	
	}


	public void gripHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kForward);
	}
	public void releaseHatchPanel() {
		hatchCoverGripper.set(DoubleSolenoid.Value.kReverse);
	}

	public void extendHatchPanel() {
	hatchCoverExtend.set(DoubleSolenoid.Value.kForward);

	}
	public void retractHatchPanel() {
		hatchCoverExtend.set(DoubleSolenoid.Value.kReverse);
	}


	public void updateStatus() {
	}
}
