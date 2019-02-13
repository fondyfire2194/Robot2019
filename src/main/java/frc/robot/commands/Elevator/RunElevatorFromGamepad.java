package frc.robot.commands.Elevator;

import frc.robot.Robot;

import frc.robot.subsystems.Elevator;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RunElevatorFromGamepad extends Command {

	public RunElevatorFromGamepad() {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.elevator);
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.elevator.elevatorMotor.configNominalOutputForward(0, 0);
		Robot.elevator.elevatorMotor.configNominalOutputReverse(-0, 0);
		Robot.elevator.elevatorMotor.configPeakOutputForward(1, 0);
		Robot.elevator.elevatorMotor.configPeakOutputReverse(-1, 0);

	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		Robot.elevator.elevatorMotor.configNominalOutputForward(0.0, 0);
		Robot.elevator.elevatorMotor.configNominalOutputReverse(0.0, 0);
		Robot.elevator.elevatorMotor.configPeakOutputForward(1, 0);
		Robot.elevator.elevatorMotor.configPeakOutputReverse(-1, 0);

		double yValue;
		double temp;
		if (Math.abs(Robot.m_oi.gamepad.getY()) > .1)
			yValue = Robot.m_oi.gamepad.getY();
		else
			yValue = 0;
		if (!Robot.elevator.elevatorOnSwitch && yValue > 0)// inhibit down move on bottom switch
			yValue = 0;
		// square joystick and preserve sign
		temp = yValue * yValue;
		if (yValue < 0)
			temp = -temp;
		Robot.elevator.runElevatorMotor(-yValue);// y up gives a negative value

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.elevator.holdPositionInches = Robot.elevator.getElevatorPositionInches();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
