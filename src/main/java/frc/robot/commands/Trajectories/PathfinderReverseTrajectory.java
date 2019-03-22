package frc.robot.commands.Trajectories;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.PathfinderReverseNotifier;
import frc.robot.Robot;
import frc.robot.SD;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class PathfinderReverseTrajectory extends Command {
	private boolean myFaceField;
	private boolean myInvertY;
	private double switchMode;
	private double startTime;

	public PathfinderReverseTrajectory(boolean faceField, boolean invertY) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		myFaceField = faceField;
		myInvertY = invertY;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();

		double P = Robot.activeTrajectoryGains[0];
		double I = 0;
		double D = Robot.activeTrajectoryGains[1];
		double V = 1 / Constants.MAX_ROBOT_FT_PER_SEC;
		double A = Robot.activeTrajectoryGains[2];

		if (myFaceField && !myInvertY)
			switchMode = 1;// normal
		if (myFaceField && myInvertY)
			switchMode = 2;// robot move fwd invert y
		if (!myFaceField && !myInvertY)
			switchMode = 3;// rev motion
		if (!myFaceField && myInvertY)
			switchMode = 4;// rev motion Y inverted

		Robot.driveTrain.revLeftDf.setTrajectory(Robot.activeLeftTrajectory);
		Robot.driveTrain.revRightDf.setTrajectory(Robot.activeRightTrajectory);

		Robot.driveTrain.revLeftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrain.revRightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrain.revLeftDf.reset();
		Robot.driveTrain.revRightDf.reset();

		Robot.trajectoryRunning = true;
		startTime = Timer.getFPGATimestamp();
		if (Robot.createTrajectoryRunFile) {
			String dirn = "REV";
			if (myInvertY)
				dirn += "IY";
			String name = Robot.trajectoryUniqueLogName + dirn + Robot.logName + ".csv";

			Robot.simpleCSVLogger2194.init(name, Robot.names, Robot.units);
			Robot.simpleCSVLogger2194.writeData(P, I, D, V, A, Robot.activeTrajectoryGains[3], 2, switchMode, 0, 0, 0,
					0, 0, 0);
		}
		PathfinderReverseNotifier.startNotifier(myFaceField, myInvertY);

	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return Robot.driveTrain.revLeftDf.isFinished() && Robot.driveTrain.revRightDf.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		PathfinderReverseNotifier.stopNotfier();
		Robot.trajectoryRunning = false;
		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);
		// Robot.driveTrain.configOpenLoopAcceleration(.5);
		if (Robot.createTrajectoryRunFile) 
		Robot.simpleCSVLogger2194.close();
		SD.putN2("Trajectory Time", Timer.getFPGATimestamp() - startTime);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
