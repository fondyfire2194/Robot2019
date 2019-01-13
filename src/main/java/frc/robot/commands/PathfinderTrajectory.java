package frc.robot.commands;

import frc.robot.PathfinderNotifier;
import frc.robot.Robot;
import frc.robot.subsystems.DriveTrain;
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
	private boolean myRobotMoveForward;

	public PathfinderTrajectory(boolean robotMoveForward) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		myRobotMoveForward = robotMoveForward;
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

		Robot.driveTrain.leftDf.setTrajectory(Robot.activeTrajectory[0]);
		Robot.driveTrain.rightDf.setTrajectory(Robot.activeTrajectory[1]);
		Robot.driveTrain.leftPositionTargetFt = Robot.activeTrajectory[0]
				.get(Robot.activeTrajectory[0].length() - 1).position;
		Robot.driveTrain.rightPositionTargetFt = Robot.activeTrajectory[1]
				.get(Robot.activeTrajectory[1].length() - 1).position;

		Robot.driveTrain.leftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrain.rightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrain.leftDf.reset();
		Robot.driveTrain.rightDf.reset();
		Robot.trajectoryRunning = true;
		if (Robot.createTrajectoryRunFile) {
			Robot.simpleCSVLogger.init("Trajectory", Robot.logName + " Fwd", Robot.names, Robot.units);
			// Robot.simpleCSVLogger.writeData(P, I, D, V, A);
		}
		scanCounter = 0;
		startTime = Timer.getFPGATimestamp();
		PathfinderNotifier.startNotifier(myRobotMoveForward);
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
		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);
		// Robot.driveTrain.configOpenLoopAcceleration(.5);
		SmartDashboard.putNumber("Trajectory Time", Timer.getFPGATimestamp() - startTime);
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
