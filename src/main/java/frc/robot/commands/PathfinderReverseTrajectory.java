package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.PathfinderReverseNotifier;
import frc.robot.Robot;

/**
 *
 */
public class PathfinderReverseTrajectory extends Command {
	private boolean myFaceField;
	private boolean myInvertY;
	public PathfinderReverseTrajectory(boolean robotMoveReverse, boolean invertY) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		myFaceField = robotMoveReverse;
		myInvertY = invertY;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.driveTrain.resetEncoders();
		// Robot.sensors.resetGyro();
		// Robot.driveTrain.configOpenLoopAcceleration(0);
		// Robot.driveTrain.configDriveNominalOut(0, driveSide.both);
		// Robot.driveTrain.configDrivePeakout(driveTrain.MAX_ROBOT_FT_PER_SEC,
		// driveSide.both);
		double P = Robot.activeTrajectoryGains[0];
		double I = 0;
		double D = Robot.activeTrajectoryGains[1];
		double V = 1 / Constants.MAX_ROBOT_FT_PER_SEC;
		double A = Robot.activeTrajectoryGains[2];

		Robot.driveTrain.revLeftDf.setTrajectory(Robot.activeTrajectory[0]);
		Robot.driveTrain.revRightDf.setTrajectory(Robot.activeTrajectory[1]);

		Robot.driveTrain.revLeftDf.configurePIDVA(P, I, D, V, A);
		Robot.driveTrain.revRightDf.configurePIDVA(P, I, D, V, A);

		Robot.driveTrain.revLeftDf.reset();
		Robot.driveTrain.revRightDf.reset();

		Robot.trajectoryRunning = true;
		if (Robot.createTrajectoryRunFile)
			Robot.simpleCSVLogger.init("Trajectory", Robot.logName + "Rev", Robot.names, Robot.units);

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
			Robot.simpleCSVLogger.close();
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
