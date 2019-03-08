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
	private String names = "Time,GyroYaw,LeftPct,RightPct,LeftOneAmps,LeftOneVolts,LeftTwoAmps,LeftTwoVolts,RightOneAmps,RightOneVolts,RightTwoAmps,RightTwoVolts,LeftFt,RightFt,LeftVel,RightVel\n";
	private String units = "mS,Deg,PCT,PCT,Amps,Volts,Amps,Volts,Amps,Volts,Amps,Volts,Ft,Ft,Ftpersec,Ftpersec\n";
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
		SmartDashboard.putString("CSVDRName",name);

		// log_name = output_dir + "log_" + name + ".csv"
		if (Robot.createDriveRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger2194.writeData((Timer.getFPGATimestamp() - startTime) * 1000,
					Robot.driveTrain.getGyroYaw(), Robot.driveTrain.leftTalonOne.getMotorOutputPercent(),
					Robot.driveTrain.rightTalonOne.getMotorOutputPercent(),
					Robot.driveTrain.leftTalonOne.getOutputCurrent(),
					Robot.driveTrain.leftTalonOne.getMotorOutputVoltage(),
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
