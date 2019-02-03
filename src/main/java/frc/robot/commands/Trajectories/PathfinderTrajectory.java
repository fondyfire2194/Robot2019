package frc.robot.commands.Trajectories;

import frc.robot.PathfinderNotifier;
import frc.robot.Robot;

import frc.robot.Constants;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class PathfinderTrajectory extends Command {
	private double startTime;
	private int scanCounter;
	private boolean myFaceField;;
	private boolean myInvertY;

	public PathfinderTrajectory(boolean faceField, boolean invertY) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myFaceField = faceField;
		myInvertY = invertY;
		requires(Robot.driveTrain);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();
		// Robot.sensors.resetGyro();
		Robot.driveTrain.leftSideStopped = false;
		Robot.driveTrain.rightSideStopped = false;

		double P = Robot.activeTrajectoryGains[0];
		double I = 0;
		double D = Robot.activeTrajectoryGains[1];
		double V = 1 / Constants.MAX_ROBOT_FT_PER_SEC;
		double A = Robot.activeTrajectoryGains[2];

		Robot.driveTrain.leftDf.setTrajectory(Robot.activeLeftTrajectory);
		Robot.driveTrain.rightDf.setTrajectory(Robot.activeRightTrajectory);
		Robot.driveTrain.leftPositionTargetFt = Robot.activeLeftTrajectory
				.get(Robot.activeLeftTrajectory.length() - 1).position;
		Robot.driveTrain.rightPositionTargetFt = Robot.activeRightTrajectory
				.get(Robot.activeRightTrajectory.length() - 1).position;

		Robot.driveTrain.leftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrain.rightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrain.leftDf.reset();
		Robot.driveTrain.rightDf.reset();
		Robot.trajectoryRunning = true;
		startTime = Timer.getFPGATimestamp();
		if (Robot.createTrajectoryRunFile) {
			Robot.simpleCSVLogger.init("Trajectory", Robot.logName + " Fwd", Robot.names, Robot.units);

		}
		scanCounter = 0;
		
		SmartDashboard.putNumber("AutoCheck", Timer.getFPGATimestamp() - startTime);
		PathfinderNotifier.startNotifier(myFaceField, myInvertY);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		scanCounter++;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return scanCounter > 20 && Robot.driveTrain.leftDf.isFinished() && Robot.driveTrain.rightDf.isFinished();
	}

	// Called once after isFinished returns true
	protected void end() {
		PathfinderNotifier.stopNotfier();
		Robot.trajectoryRunning = false;
		Robot.driveTrain.leftDf.reset();
		Robot.driveTrain.rightDf.reset();
		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);
		// Robot.driveTrain.configOpenLoopAcceleration(.5);
		SmartDashboard.putNumber("Trajectory Time", Timer.getFPGATimestamp() - startTime);
		
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
