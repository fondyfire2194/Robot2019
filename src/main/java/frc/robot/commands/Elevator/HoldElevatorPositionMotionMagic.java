package frc.robot.commands.Elevator;

import frc.robot.Robot;
import frc.robot.Pref;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HoldElevatorPositionMotionMagic extends Command {
	private double lastHoldPositionInches;

	public HoldElevatorPositionMotionMagic() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.elevator.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		Robot.elevator.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		Robot.elevator.elevatorMotor.selectProfileSlot(0, 0);

		Robot.elevator.elevatorMotor.configOpenloopRamp(0, 0);
		Robot.elevator.elevatorMotor.configClosedloopRamp(0, 0);
		Robot.elevator.elevatorMotor.configPeakOutputForward(1, 0);
		Robot.elevator.elevatorMotor.configPeakOutputReverse(-1, 0);
		Robot.elevator.elevatorMotor.configNominalOutputForward(0, 0);
		Robot.elevator.elevatorMotor.configNominalOutputReverse(0, 0);
		Robot.elevator.elevatorMotor.selectProfileSlot(0, 0);

		
			Robot.elevator.elevatorMotor.config_kF(0,
					Pref.getPref("ElevatorMMKf"),0);
			Robot.elevator.elevatorMotor.config_kP(0,
			Pref.getPref("ElevatorMMKp"),  0);
			Robot.elevator.elevatorMotor.config_kI(0,
			Pref.getPref("ElevatorMMKi"), 0);
			Robot.elevator.elevatorMotor.config_kD(0,
			Pref.getPref("ElevatorMMKd"), 0);


		Robot.elevator.holdPositionInches = Robot.elevator.getElevatorPositionInches();
		Robot.elevator.lastHoldPositionInches = Robot.elevator.holdPositionInches + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.elevator.holdPositionInches != lastHoldPositionInches) {

			// } else {
			// 	Robot.elevator.elevatorMotor.config_kF(0,
			// 	Pref.getPref("ElevatorDownMMKf"), 0);
			// 	Robot.elevator.elevatorMotor.config_kP(0,
			// 	Pref.getPref("ElevatorDownMMKp"), 0);
			// 	Robot.elevator.elevatorMotor.config_kI(0,
			// 	Pref.getPref("ElevatorDownMMKi"), 0);
			// 	Robot.elevator.elevatorMotor.config_kD(0,
			// 	Pref.getPref("ElevatorDownMMKd"), 0);
			// }
			Robot.elevator.magicMotionElevator(Robot.elevator.holdPositionInches,
					Constants.ELEVATOR_POSITION_RATE);
			lastHoldPositionInches = Robot.elevator.holdPositionInches;
		}
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
