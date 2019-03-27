package frc.robot.commands.Cargo;

import frc.robot.Robot;
import frc.robot.subsystems.GamePieceHandler;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class LogCargoMotor extends TimedCommand {
	private double startTime;
	private String names = "Current,Output,Bus\n";
	private String units = "Amps,Pct,Volts\n";
	String output_dir = "/U" + "/data_capturesDSMKE/Climber/"; // USB drive is mounted to /U on roboRIO
	String name1 = "Climber";
	String name = output_dir + name1;

	public LogCargoMotor(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time

	protected void initialize() {
		// double temp = (int) Timer.getFPGATimestamp();
		// name += String.valueOf(temp) + ".csv";
		name = Robot.climberUniqueLogName;
		SmartDashboard.putString("CSVDRName", name);

		// log_name = output_dir + "log_" + name + ".csv"
		if (Robot.createDriveRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			Robot.simpleCSVLogger2194.writeData(GamePieceHandler.cargoMotor.getOutputCurrent(),
					GamePieceHandler.cargoMotor.getMotorOutputPercent(),GamePieceHandler.cargoMotor.getBusVoltage());
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
