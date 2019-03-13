package frc.robot;

import edu.wpi.first.wpilibj.Notifier;

import jaci.pathfinder.Pathfinder;

public class PathfinderNotifier {

	public static double timeAverage;
	public static boolean isRunning;
	public static int segmentCounter = 0;
	public static int notifierRunning;
	private static double headingMultiplier;

	public static final class PeriodicRunnable implements java.lang.Runnable {
		public void run() {
			runTrajectory();
		}
	}

	static Notifier _notifier = new Notifier(new PeriodicRunnable());
	private static int activeTrajectoryLength;
	private static double periodic_time = .02;
	private static double desired_heading;
	private static boolean myFaceField;
	private static boolean myInvertY;
	private static int switchMode;

	public static void startNotifier(boolean faceField, boolean invertY) {
		/**
		 * all moves are towards the field and away from the wall robot can move in its
		 * forward or reverse directiom it can have its command angle inverted to mirror
		 * Y it can have its sides inverted by following opposite side trajectory
		 * 
		 */
		myFaceField = faceField;
		myInvertY = invertY;
		if (myFaceField && !myInvertY)
			switchMode = 1;// normal
		if (myFaceField && myInvertY)
			switchMode = 2;// robot move fwd invert y
		if (!myFaceField && !myInvertY)
			switchMode = 3;// rev motion
		if (!myFaceField && myInvertY)
			switchMode = 4;// rev motion Y inverted
		headingMultiplier = 1;
		if (Constants.usePathWeaver)
			headingMultiplier = -1;

		segmentCounter = 0;
		activeTrajectoryLength = Robot.activeLeftTrajectory.length();
		periodic_time = .02;// Robot.driveTrain.leftDf.getSegment().dt;
		_notifier.startPeriodic(periodic_time);
	}

	public static void stopNotfier() {
		_notifier.stop();
	}

	private static void runTrajectory() {
		/*
		 * Pathfinder calculations to arrive at pct output to left and right motors
		 * 
		 * Note the gyro yaw is opposite in sign to the Pathfinder heading so it needs
		 * to be negated before it is subtracted to get the angular error. A positive
		 * error means turn left so speed up the right side and slow down the left The
		 * negative gain multiplier causes (right - turn) and (left + turn) to do this
		 * 
		 * public double calculate(double distance_covered)
		 * 
		 * { if (segment <trajectory.length()) {
		 * 
		 * Trajectory.Segment seg = trajectory.get(segment);
		 * 
		 * double error = seg.position - distance_covered;
		 * 
		 * double calculated_value = kp* error + // Proportional
		 *
		 * kd * ((error - last_error) / seg.dt) + //Derivative
		 * 
		 * (kv * seg.velocity + ka * seg.acceleration); // V and A Terms
		 *
		 * last_error = error; heading = seg.heading; segment++;
		 *
		 * return calculated_value;
		 * 
		 * }
		 * 
		 * else return 0; }
		 *
		 */
		segmentCounter++;
		Robot.currentTrajectorySegment = segmentCounter;
		double left = 0;
		double right = 0;
		double leftPct = 0;
		double rightPct = 0;
		double angleDifference = 0;
		double turn = 0;
		// convenience because gyro action is opposite of trajectory generation
		double correctedGyroYaw = -Robot.driveTrain.getGyroYaw();

		switch (switchMode) {

		case 1:
			/**
			 * normal condition robot moves forward to field standard Pathfinder trajectory
			 * use
			 */
			left = Robot.driveTrain.leftDf.calculate(Robot.driveTrain.getLeftFeet());
			right = Robot.driveTrain.rightDf.calculate(Robot.driveTrain.getRightFeet());
			desired_heading = headingMultiplier * Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = Constants.MINIMUM_START_PCT + left + turn;
			rightPct = Constants.MINIMUM_START_PCT + right - turn;
			break;

		case 2:
			/**
			 * allows one trajectory to be used instead of two. Used for opposite side of
			 * cargo ship. Robot moves forward to field with y inverted. Left/right follow
			 * right/left distance follower and target angle inverted
			 * 
			 */
			right = Robot.driveTrain.leftDf.calculate(Robot.driveTrain.getRightFeet());
			left = Robot.driveTrain.rightDf.calculate(Robot.driveTrain.getLeftFeet());
			desired_heading = headingMultiplier * (-Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading()));
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = Constants.MINIMUM_START_PCT + left + turn;
			rightPct = Constants.MINIMUM_START_PCT + right - turn;
			break;

		case 3:
			/**
			 * robot moves backwards to field This requires a negated drive command and the
			 * side positions must be negated and exchanged. The side exchange is needed for
			 * turns as the one with the longer distance is now the opposite The target
			 * angle doesn't change but if this is the first move after startup, then the
			 * gyro angle will be 180 off from normal and must be compensated somehow for
			 * any future motions
			 * 
			 */
			right = Robot.driveTrain.leftDf.calculate(-Robot.driveTrain.getRightFeet());
			left = Robot.driveTrain.rightDf.calculate(-Robot.driveTrain.getLeftFeet());
			desired_heading = headingMultiplier * Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading());
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;

			leftPct = -(Constants.MINIMUM_START_PCT + left - turn);
			rightPct = -(Constants.MINIMUM_START_PCT + right + turn);

			break;

		case 4:
			/**
			 * robot moves backwards to field with Y invertd. This requires a negated drive
			 * command. Side positions must be negated but not exchanged.The target angle
			 * doesn't change but if this is the first move after startup, then the gyro
			 * angle will be 180 off from normal and must be compensated somehow for any
			 * future motions
			 * 
			 */
			right = Robot.driveTrain.rightDf.calculate(-Robot.driveTrain.getRightFeet());
			left = Robot.driveTrain.leftDf.calculate(-Robot.driveTrain.getLeftFeet());
			desired_heading = headingMultiplier * (-Pathfinder.r2d(Robot.driveTrain.leftDf.getHeading()));
			angleDifference = Pathfinder.boundHalfDegrees(desired_heading - correctedGyroYaw);
			turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;
			leftPct = -(Constants.MINIMUM_START_PCT + left - turn);
			rightPct = -(Constants.MINIMUM_START_PCT + right + turn);
			break;

		default:
			break;
		}
		Robot.driveTrain.leftDriveOut(leftPct);
		Robot.driveTrain.rightDriveOut(rightPct);

		if (segmentCounter < activeTrajectoryLength - 1) {
			/*
			 * write linear and angular data to file
			 * 
			 * names = { "Step", "Left Cmd", "Left Ft", "Right Cmd ", "Right Ft",
			 * "Angle Cmd", "Angle", "LeftSegVel", "left", "ActLeftVel", "RightSegVel",
			 * "right", "ActRightVel", "turn"};
			 * 
			 */

			Robot.simpleCSVLogger2194.writeData((double) segmentCounter, Robot.driveTrain.leftDf.getSegment().position,
					Robot.driveTrain.getLeftFeet(), Robot.driveTrain.rightDf.getSegment().position,
					Robot.driveTrain.getRightFeet(), Pathfinder.boundHalfDegrees(desired_heading),
					Robot.driveTrain.getGyroYaw(),
					Robot.driveTrain.leftDf.getSegment().velocity / Constants.MAX_ROBOT_FT_PER_SEC, leftPct,
					Robot.driveTrain.getLeftFeetPerSecond() / Constants.MAX_ROBOT_FT_PER_SEC,
					Robot.driveTrain.rightDf.getSegment().velocity / Constants.MAX_ROBOT_FT_PER_SEC, rightPct,
					Robot.driveTrain.getRightFeetPerSecond() / Constants.MAX_ROBOT_FT_PER_SEC, turn);
		}

	}

}
