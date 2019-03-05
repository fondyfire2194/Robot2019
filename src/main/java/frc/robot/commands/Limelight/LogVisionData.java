package frc.robot.commands.Limelight;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;

/**
 *
 */
public class LogVisionData extends TimedCommand {
	private double startTime;
	private String names = "Time,LeftFt,RightFt,SpeedFPS,GyroYaw,BoxHt,BoxWdth,HzAngle,VertAngle,TgtSen\n";
	private String units = "mS,Ft,Ft,FPS,Deg,Px,Px,Deg,Deg,T_F\n";
	String output_dir = "/U" + "/data_capturesDS19/Vision/"; // USB drive is mounted to /U on roboRIO
String name1 = "Vision";
String name = output_dir + name1;
	public LogVisionData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double temp= (int)Timer.getFPGATimestamp();
		name+=  String.valueOf(temp)  + ".csv";

		if (Robot.createVisionRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createVisionRunFile) {
			double targetFound = 0.0;
			if (Robot.limelightCamera.getIsTargetFound())
				targetFound = 1.0;
			Robot.simpleCSVLogger2194.writeData((Timer.getFPGATimestamp() - startTime), Robot.driveTrain.getLeftFeet(),
					Robot.driveTrain.getRightFeet(), Robot.driveTrain.getLeftFeetPerSecond(),
					Robot.driveTrain.getGyroYaw(), Robot.limelightCamera.getBoundingBoxHeight(),
			  	    Robot.limelightCamera.getBoundingBoxWidth(),Robot.visionData.calculateDistance(),
					Robot.limelightCamera.getdegRotationToTarget(),Robot.limelightCamera.getdegVerticalToTarget(), targetFound);
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createVisionRunFile)
			Robot.simpleCSVLogger2194.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
