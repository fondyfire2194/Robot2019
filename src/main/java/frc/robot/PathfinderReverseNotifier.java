package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import jaci.pathfinder.Pathfinder;

public class PathfinderReverseNotifier {
	private static int passCounter = 0;
	private static int activeTrajectoryLength;
	private static double periodic_time = .02;
	private static double desired_heading;

	public static final class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			runReverseTrajectory();
		}
	}

	static Notifier _notifier = new Notifier(new PeriodicRunnable());
	private static double lastSegmentPosition;
	private static boolean myRobotMoveReverse;
	private static boolean myInvertY;
	private static int switchMode;

	public static void startNotifier(boolean robotMoveReverse, boolean invertY) {
				/**
		 * all moves are towards the wall and away from the field. Robot can move in its
		 * forward or reverse direction. It can have its command angle inverted to mirror
		 * Y it can have its sides inverted by following opposite side trajectory
		 * 
		 */

		myRobotMoveReverse = robotMoveReverse;
		myInvertY = invertY;
		if (!myRobotMoveReverse && !myInvertY)
			switchMode = 1;// reverse toward wall
		if (!myRobotMoveReverse && myInvertY)
			switchMode = 2;// reverse toward wall invert y
		if (myRobotMoveReverse && !myInvertY)
			switchMode = 3;// rev toward field
		if (myRobotMoveReverse && myInvertY)
			switchMode = 4;// rev motion Y inverted


		activeTrajectoryLength = Robot.activeTrajectory[0].length();
		lastSegmentPosition = Robot.activeTrajectory[0].get(activeTrajectoryLength - 1).position;
		passCounter = activeTrajectoryLength - 1;
		periodic_time = Robot.driveTrain.revLeftDf.getSegment().dt;
		_notifier.startPeriodic(periodic_time);
	}

	public static void stopNotfier() {
		_notifier.stop();
	}

	/*
	 * Fwd math
	 * 
	 * leftPct = driveTrain.MINIMUM_START_PCT + left + turn;
	 * 
	 * rightPct = driveTrain.MINIMUM_START_PCT + right - turn;
	 * 
	 * a + turn reading means robot is turned cw so need to speed up right and slow
	 * down left
	 * 
	 * Rev math
	 * 
	 * in reverse a + turn still means robot is turned cw but now we need to speed
	 * up left and slow down right so turn needs to be negated in equations Also
	 * output pct needs to be negated as does position feedback reading.
	 * 
	 * leftPct = -(driveTrain.MINIMUM_START_PCT + left - turn);
	 * 
	 * rightPct = -(driveTrain.MINIMUM_START_PCT + right + turn);
	 * 
	 */
	private static void runReverseTrajectory() {
		passCounter--;
		double left = 0;
		double right = 0;
		//convenience because gyro action is opposite of trajectory generation
		double correctedGyroYaw = -Robot.driveTrain.getGyroYaw();

		switch (switchMode) {

		case 1:
		/**
		 *  normal condition robot moves forward to field
		 * standard Pathfinder trajectory use
		 *  */
	    	double left = Robot.driveTrainCanBus.revLeftDf.calculate(-Robot.driveTrainCanBus.getLeftFeet());
		    double right = Robot.driveTrainCanBus.revRightDf.calculate(-Robot.driveTrainCanBus.getRightFeet());
			desired_heading = Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = -(Constants.MINIMUM_START_PCT + left - turn);
			rightPct = -(Constants.MINIMUM_START_PCT + right + turn);
			break;

		case 2:
			/**
			 * allows one trajectory to be used instead of two. Used for opposite side of
			 * cargo ship. Robot moves forward to field with y inverted. Left/right follow
			 * right/left distance follower and target angle inverted
			 * 
			 */
			right = Robot.driveTrain.revLeftDf.calculate(Robot.driveTrain.getRightFeet());
			left = Robot.driveTrain.revRightDf.calculate(Robot.driveTrain.getLeftFeet());
			desired_heading = -Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = Constants.MINIMUM_START_PCT + left + turn;
			rightPct = Constants.MINIMUM_START_PCT + right - turn;
			break;

		case 3:
			/**
			 * robot moves backwards to field This requires a negated drive command and the
			 * side positions nust be negated and exchanged. The side exchange is needed for
			 * turns as the one with the longer distance is now the opposite The target
			 * angle doesnt change but if this is the first move after startup, then the
			 * gyro angle will be 180 off from normal and must be compensated somehow for
			 * any future motions
			 * 
			 */
			right = Robot.driveTrain.revLeftDf.calculate(-Robot.driveTrain.getLeftFeet());
			left = Robot.driveTrain.revRightDf.calculate(-Robot.driveTrain.getRightFeet());
			desired_heading = Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = -(Constants.MINIMUM_START_PCT + left + turn);
			rightPct = -(Constants.MINIMUM_START_PCT + right - turn);
			break;

		case 4:
		/**
			 * robot moves backwards to field with Y invertd. This requires a negated drive command.
			 * Side positions nust be negated but not exchanged.The target
			 * angle doesnt change but if this is the first move after startup, then the
			 * gyro angle will be 180 off from normal and must be compensated somehow for
			 * any future motions
			 * 
			 */
			right = Robot.driveTrain.revLeftDf.calculate(-Robot.driveTrain.getRightFeet());
			left = Robot.driveTrain.revRightDf.calculate(-Robot.driveTrain.getLeftFeet());
			desired_heading = -Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = -(Constants.MINIMUM_START_PCT + left + turn);
			rightPct = -(Constants.MINIMUM_START_PCT + right - turn);
			break;

		default:
			break;
		}

		if (myRobotMoveReverse) {
			left = Robot.driveTrain.revLeftDf.calculate(-Robot.driveTrain.getRightFeet());
			right = Robot.driveTrain.revRightDf.calculate(-Robot.driveTrain.getLeftFeet());
		} else {
			left = Robot.driveTrain.revLeftDf.calculate(-Robot.driveTrain.getLeftFeet());
			right = Robot.driveTrain.revRightDf.calculate(-Robot.driveTrain.getRightFeet());
		}



		Robot.driveTrain.leftDriveOut(leftPct);
		Robot.driveTrain.rightDriveOut(rightPct);

		if (passCounter > 1) {
			/*
			 * names = { "Step", "Left Cmd", "Left Ft", "Right Cmd ", "Right Ft",
			 * "Angle Cmd", "Angle", "LeftSegVel", "left", "ActLeftVel", "RightSegVel",
			 * "right", "ActRightVel", "turn"};
			 * 
			 */
			Robot.simpleCSVLogger.writeData((double) passCounter,
					lastSegmentPosition - Robot.driveTrain.revLeftDf.getSegment().position,
					-Robot.driveTrain.getLeftFeet(),
					lastSegmentPosition - Robot.driveTrain.revRightDf.getSegment().position,
					-Robot.driveTrain.getRightFeet(), Pathfinder.boundHalfDegrees(desired_heading),
					-Robot.driveTrain.getGyroYaw(),
					Robot.driveTrain.revLeftDf.getSegment().velocity / Constants.MAX_ROBOT_FT_PER_SEC, leftPct,
					Robot.driveTrain.getLeftFeetPerSecond() / Constants.MAX_ROBOT_FT_PER_SEC,
					Robot.driveTrain.revRightDf.getSegment().velocity / Constants.MAX_ROBOT_FT_PER_SEC, rightPct,
					Robot.driveTrain.getRightFeetPerSecond() / Constants.MAX_ROBOT_FT_PER_SEC, turn);
		}
	}
}
