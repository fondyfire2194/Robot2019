package frc.robot.commands.Elevator;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class LogElevatorData extends TimedCommand {
	private double startTime;
	private String names = "Time,TrajPos,TrajVel,Tgt Ht,Act Ht,Amps,Volts, Speed\n";
	private String units = "mS,Inches,Inches,Inches,Inches,Amps,Volts,In/Sec\n";
	String output_dir = "/U" + "/data_capturesDS19/Elevator/"; // USB drive is mounted to /U on roboRIO
	String name1 = "Elevator";
	String name = output_dir + name1;

	public LogElevatorData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double temp = (int) Timer.getFPGATimestamp();
		name += String.valueOf(temp) + ".csv";

		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createElevatorRunFile) {
			Robot.simpleCSVLogger2194.writeData((Timer.getFPGATimestamp() - startTime),
					frc.robot.Robot.elevator.elevatorMotor.getActiveTrajectoryPosition(),
					Robot.elevator.elevatorMotor.getActiveTrajectoryVelocity(), Robot.elevator.holdPositionInches,
					Robot.elevator.getElevatorPositionInches(), Robot.elevator.elevatorMotor.getOutputCurrent(),
					Robot.elevator.elevatorMotor.getMotorOutputVoltage(),
					Robot.elevator.getElevatorSpeedInchesPerSecond());
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createElevatorRunFile)
			Robot.simpleCSVLogger2194.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
