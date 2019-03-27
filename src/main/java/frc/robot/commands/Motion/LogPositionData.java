package frc.robot.commands.Motion;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.TimedCommand;
import frc.robot.Pref;

/**
 *
 */
public class LogPositionData extends TimedCommand {
	private double startTime;
	private String names = "Time,LeftFt,RightFt,LeftFPS,RightFPS,LeftMOP,RightMOP,LeftFE,RightFE,GyroYaw,BoxHt,BoxWdth,VertAngle,TgtSeen,DRStAn,USND,useGyComp,ActComp\n";
	private String units = "mS,Ft,Ft,FPS,FPS,Pct,Pct,CP100,CP100,Deg,Px,Px,Deg,T_F,Deg,USNDT_F,Pct\n";
	String output_dir = "/U" + "/data_capturesDSMKE/Vision/"; // USB drive is mounted to /U on roboRIO
	String name1 = "Position";
	String name = output_dir + name1;

	public LogPositionData(double timeout) {
		super(timeout);
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		double temp = (int) Timer.getFPGATimestamp();
		name += String.valueOf(temp) + ".csv";

		if (Robot.createDriveRunFile)
			Robot.simpleCSVLogger2194.init(name, names, units);
		startTime = Timer.getFPGATimestamp();
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		if (Robot.createDriveRunFile) {
			double targetFound = 0.0;
			double useGyroComp = 0.0;
			if (Robot.limelightCamera.getIsTargetFound())
				targetFound = 1.0;
			if (Robot.driveTrain.useGyroComp)
				useGyroComp = 1.0;
			Robot.simpleCSVLogger2194.writeData((Timer.getFPGATimestamp() - startTime), Robot.driveTrain.getLeftFeet(),
					Robot.driveTrain.getRightFeet(), Robot.driveTrain.getLeftFeetPerSecond(),
					Robot.driveTrain.getRightFeetPerSecond(), Robot.driveTrain.getLeftCommand(),
					Robot.driveTrain.getRightCommand(), Robot.driveTrain.getGyroYaw(),
					Robot.driveTrain.leftTalonOne.getClosedLoopError(),
					Robot.driveTrain.rightTalonOne.getClosedLoopError(), Robot.limelightCamera.getBoundingBoxHeight(),
					Robot.limelightCamera.getBoundingBoxWidth(), Robot.limelightCamera.getdegVerticalToTarget(),
					targetFound, Robot.driveTrain.driveStraightAngle, Robot.ultrasound.getDistanceInches(), useGyroComp,
					Robot.driveTrain.activeMotionComp);
		}
	}

	// Called once after timeout
	protected void end() {
		if (Robot.createDriveRunFile)
			Robot.simpleCSVLogger2194.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}
}
