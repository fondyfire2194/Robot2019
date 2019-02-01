package frc.robot.commands.Motion;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * 
 */
public class LogDriveData extends Command {
	private double startTime, myTimeout;
	private String myTypeName;
	private String mySubDir;

	private String[] names = { "Time", "Gyro Yaw", "LeftPct", "RightPct", "LeftOne Amps", "LeftOne Volts",
			"LeftTwo Amps", "LeftTwo Volts", "RightA Amps", "RightA Volts", "RightB Amps", "RightB Volts", "Left Ft",
			"Right Ft", "Left Vel", "Right Vel" };
	private String[] units = { "mS", "Degrees", "PU", "PU", "Amps", "Volts", "Amps", "Volts", "Amps", "Volts", "Amps",
			"Volts", "Ft", "Ft", "Ft/sec", "Ft/sec"};

	public LogDriveData(String subDir, String typeName, double timeout) {
		myTimeout = timeout;
		myTypeName = typeName;
		mySubDir = subDir;
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		setTimeout(myTimeout);
		Robot.createTrajectoryRunFile = false;
		Robot.createDriveRunFile = true;
		Robot.simpleCSVLogger.init(mySubDir, myTypeName, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger.writeData((Timer.getFPGATimestamp() - startTime) * 1000,
					Robot.driveTrain.getGyroYaw(), Robot.driveTrain.leftTalonOne.getMotorOutputPercent(),
					Robot.driveTrain.rightTalonOne.getMotorOutputPercent(),
					Robot.driveTrain.leftTalonOne.getOutputCurrent(),
					Robot.driveTrain.leftTalonTwo.getMotorOutputVoltage(),
					Robot.driveTrain.leftTalonTwo.getOutputCurrent(),
					Robot.driveTrain.leftTalonTwo.getMotorOutputVoltage(),
					Robot.driveTrain.rightTalonOne.getOutputCurrent(),
					Robot.driveTrain.rightTalonOne.getMotorOutputVoltage(),
					Robot.driveTrain.rightTalonTwo.getOutputCurrent(),
					Robot.driveTrain.rightTalonTwo.getMotorOutputVoltage(), Robot.driveTrain.getLeftFeet(),
					Robot.driveTrain.getRightFeet(), Robot.driveTrain.getLeftFeetPerSecond(),
					Robot.driveTrain.getRightFeetPerSecond()
					// Robot.pdp.getVoltage());
			);
		}
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut() || !Robot.createDriveRunFile;
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createDriveRunFile && Robot.simpleCSVLogger.log_open) {
			Robot.simpleCSVLogger.close();
			Robot.createDriveRunFile = false;
		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
