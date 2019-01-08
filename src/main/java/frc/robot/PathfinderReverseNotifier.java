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

	public static void startNotifier() {
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
		double left = Robot.driveTrain.revLeftDf.calculate(-Robot.driveTrain.getLeftFeet());
		double right = Robot.driveTrain.revRightDf.calculate(-Robot.driveTrain.getRightFeet());

		desired_heading = Pathfinder.r2d(Robot.driveTrain.revLeftDf.getHeading());

		double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - (-Robot.driveTrain.getGyroYaw()));
		double turn = Robot.activeTrajectoryGains[3] * (-1.0 / 80.0) * angleDifference;

		double leftPct = Constants.MINIMUM_START_PCT + left - turn;
		double rightPct = Constants.MINIMUM_START_PCT + right + turn;

		Robot.driveTrain.leftDriveOut(-leftPct);
		Robot.driveTrain.rightDriveOut(-rightPct);

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
