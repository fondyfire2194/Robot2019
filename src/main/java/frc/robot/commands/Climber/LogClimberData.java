package frc.robot.commands.Climber;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * 
 */
public class LogClimberData extends TimedCommand {
	private double startTime;
	private String names = "LegCmd,LegIPS,ArmCmd,ArmDPS,LegAmps,LegPosition,ArmAmps,ArmDegrees,DriveAmps,Pitch";
	private String units = "%,IPS,%,DPS,Amps,Inches,Amps,Angle,Amps,Degrees";
	String output_dir = "/U" + "/data_capturesDS19/Climber/"; // USB drive is mounted to /U on roboRIO
	String name1 = "Climber";
	String name = output_dir + name1;

	public LogClimberData(double timeout) {
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
			Robot.simpleCSVLogger2194.writeData(Robot.climberLeg.getLegPercentOut(),
					Robot.climberLeg.getLegInPerSec(), Robot.climberArm.getArmPercentOut(),
					Robot.climberArm.getArmSpeedDegPerSec(), Robot.climberLeg.getLegCurrent(),
					Robot.climberLeg.getLegInches(), Robot.climberArm.getArmCurrent(),
					Robot.climberArm.getArmDegrees(), Robot.climberDrive.getDriveCurrent(),
					Robot.driveTrain.getGyroPitch());
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
