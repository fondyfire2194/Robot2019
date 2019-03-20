package frc.robot.commands.Motion;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class LogDriveData extends TimedCommand {
	private double startTime;
	private String names = "LeftCmd,LeftFPS,RightCmd,RightFPS,LeftOneAmps,LeftTwoAmps,RightOneAmps,RightTwoAmps,lcp,rcp";
	private String units = "PU,FPS,PU,FPS,Amps,Amps,Amps,Amps,cp100,cp100";
	String output_dir = "/U" + "/data_capturesDS19/Drive/"; // USB drive is mounted to /U on roboRIO
	String name1 = "Drive";
	String name = output_dir + name1;

	public LogDriveData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time

	protected void initialize() {
		// double temp = (int) Timer.getFPGATimestamp();
		// name += String.valueOf(temp) + ".csv";
		name = Robot.driveUniqueLogName;
		SmartDashboard.putString("CSVDRName", name);

		// log_name = output_dir + "log_" + name + ".csv"
		if (Robot.createDriveRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger2194.writeData(Robot.driveTrain.getLeftCommand(),
					Robot.driveTrain.getLeftFeetPerSecond(), Robot.driveTrain.getRightCommand(),
					Robot.driveTrain.getRightFeetPerSecond(), Robot.driveTrain.leftTalonOne.getOutputCurrent(),
					Robot.driveTrain.leftTalonTwo.getOutputCurrent(), Robot.driveTrain.rightTalonOne.getOutputCurrent(),
					Robot.driveTrain.rightTalonTwo.getOutputCurrent(),
					Robot.driveTrain.leftTalonOne.getSelectedSensorVelocity(0),
					Robot.driveTrain.rightTalonOne.getSelectedSensorVelocity(0));
		}
	}

	@Override
	protected boolean isFinished() {
		return isTimedOut() || !Robot.createDriveRunFile;
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger2194.close();

		}
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
