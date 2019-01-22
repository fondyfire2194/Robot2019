package frc.robot.commands.Elevator;

import frc.robot.Robot;
import frc.robot.Pref;
import frc.robot.subsystems.Elevator;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class HoldElevatorPositionMotionMagic extends Command {
	private double lastHoldPositionInches;
	private boolean firstTime;

	public HoldElevatorPositionMotionMagic() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);

	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Elevator.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, 0);
		Elevator.elevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 0);
		Elevator.elevatorMotor.selectProfileSlot(0, 0);

		Elevator.elevatorMotor.configOpenloopRamp(0, 0);
		Elevator.elevatorMotor.configClosedloopRamp(0, 0);
		Elevator.elevatorMotor.configPeakOutputForward(1, 0);
		Elevator.elevatorMotor.configPeakOutputReverse(-1, 0);
		Elevator.elevatorMotor.configNominalOutputForward(0, 0);
		Elevator.elevatorMotor.configNominalOutputReverse(0, 0);

		Robot.elevator.holdPositionInches = Robot.elevator.getElevatorPositionInches();
		Robot.elevator.lastHoldPositionInches = Robot.elevator.holdPositionInches + .01;

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		if (Robot.elevator.holdPositionInches != lastHoldPositionInches) {

			Elevator.elevatorMotor.selectProfileSlot(0, 0);

			if (Robot.elevator.moveIsUp) {
				Elevator.elevatorMotor.config_kF(0,
						Pref.getPref("ElevatorMMKf"),0);
				Elevator.elevatorMotor.config_kP(0,
				Pref.getPref("ElevatorMMKp"),  0);
				Elevator.elevatorMotor.config_kI(0,
				Pref.getPref("ElevatorMMKi"), 0);
				Elevator.elevatorMotor.config_kD(0,
				Pref.getPref("ElevatorMMKd"), 0);
			} else {
				Elevator.elevatorMotor.config_kF(0,
				Pref.getPref("ElevatorDownMMKf"), 0);
				Elevator.elevatorMotor.config_kP(0,
				Pref.getPref("ElevatorDownMMKp"), 0);
				Elevator.elevatorMotor.config_kI(0,
				Pref.getPref("ElevatorDownMMKi"), 0);
				Elevator.elevatorMotor.config_kD(0,
				Pref.getPref("ElevatorDownMMKd"), 0);
			}
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
