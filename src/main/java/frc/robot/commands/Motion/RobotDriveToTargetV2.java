package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import frc.robot.LimelightControlMode.*;

/**
 * This command is used from a middle start where the distance and angle is
 * known precisely, but also following a rotate which followed a trajectory. In
 * this latter case, distance to the target is subject to variaton of possibly
 * as much as +/- 1/2 foot ? The distance can be up to 11 feet ? when
 * approaching the load station from Cargo Ship 2, and 10 ft from cargo ship
 * end. Delivery to cargo ship distances are smaller in the range of 5 ft.
 * Angles may be off by +/- 5 degrees which is much less of a concern. Good
 * vision distance resolution starts at around 7 feet. The position loop
 * slowdown starts at 2.5 feet. Positioning speed is 8 ft per second; Speed 1
 * foot away would be 8/2.5 or 3 ft per sec so it would hit at that speed with a
 * 1 foot error. The robot can run into the physical stops of the load station /
 * cargo ship. The hatch cover pusher cylinders could be left extended and
 * sensed being pushed back. This is short range and needs traveling wires.
 * 
 * 
 * 
 * 
 */
public class RobotDriveToTargetV2 extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private boolean useGyroComp;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	public double myDistance;
	private double myEndpoint;
	private double startOfVisionFt;
	private double endOfVisionFt;
	private boolean inVisionRange;
	private double remainingFtToHatch;
	private double myTargetAngle;
	private boolean targetWasSeen;
	private int targetNotSeenCtr;
	private double visionTurnGain;
	private double Kp;
	private double Kd;
	private double calcLoopSpeed;
	private double loopTime = .02;
	private double lastRemainingDistance;
	private double positionChange;
	private double positionRateOfChange;
	private boolean visionTargetSeen;
	private double robotDistance;

	/**
	 * kp equivalent is the speed / slowdown feet or 7.5 ft/sec/2.5ft from previous
	 * testing = 3 So at 2 ft speed is 6 ft per sec, at 1 ft it is 3
	 */

	public RobotDriveToTargetV2(double distance, double speed, double targetAngle, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myDistance = distance;
		myTimeout = timeout;
		myTargetAngle = targetAngle;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		Robot.limelightCamera.setLEDMode(LedMode.kforceOn);
		Robot.driveTrain.resetEncoders();
		rampIncrement = mySpeed / 25;
		setTimeout(myTimeout);
		Robot.isPositioning = true;
		currentMaxSpeed = 0;
		doneAccelerating = false;
		Robot.activeMotionComp = 0.;
		Robot.driveTrain.driveStraightAngle = myTargetAngle;
		targetWasSeen = false;
		targetNotSeenCtr = 0;
		Robot.noCameraTargetFound = false;
		lastRemainingDistance = 0;
		// "linear" vision range in ft
		startOfVisionFt = Robot.visionData.startOfVisionFt;
		endOfVisionFt = Robot.visionData.endOfVisionFt;
		visionTurnGain = Pref.getPref("VisionKp");
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		doAccel();
		/**
		 * capture data for this cycle
		 * 
		 */
		visionTargetSeen = Robot.limelightCamera.getIsTargetFound();
		robotDistance = Robot.driveTrain.getLeftFeet();
		remainingFtToHatch = myEndpoint - robotDistance;

		inVisionRange = (remainingFtToHatch < startOfVisionFt && remainingFtToHatch > endOfVisionFt);

		positionChange = lastRemainingDistance - remainingFtToHatch;
		lastRemainingDistance = robotDistance;
		positionRateOfChange = positionChange / loopTime;

		calcLoopSpeed = remainingFtToHatch * Kp + positionRateOfChange * Kd;

		if (calcLoopSpeed > mySpeed)
			currentMaxSpeed = mySpeed;
		else
			currentMaxSpeed = calcLoopSpeed;

		// set minimum speed
		if (currentMaxSpeed < .2) {
			currentMaxSpeed = .2;
		}

		// in vision zone keep gyro target angle current in case need to switch
		// over to gyro

		if (inVisionRange)
			Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();

		if (inVisionRange & !targetWasSeen)
			targetNotSeenCtr++;

		// if (targetNotSeenCtr > 10)
		// Robot.noCameraTargetFound = true;

		if (visionTargetSeen)
			targetWasSeen = true;

		Robot.useVisionComp = inVisionRange && visionTargetSeen;

		useGyroComp = !Robot.useVisionComp;

		if (Robot.useVisionComp) {
			if (Robot.limelightOnEnd) {
				Robot.activeMotionComp = Robot.limelightCamera.getdegVerticalToTarget() * visionTurnGain;
			} else {
				Robot.activeMotionComp = Robot.limelightCamera.getdegRotationToTarget() * visionTurnGain;
			}
		}
		if (useGyroComp) {
			Robot.activeMotionComp = Robot.driveTrain.getCurrentComp();
		}
		Robot.driveTrain.arcadeDrive(currentMaxSpeed * Constants.FT_PER_SEC_TO_PCT_OUT, Robot.activeMotionComp);
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.driveTrain.getLeftFeet() >= myEndpoint;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		// Robot.autonomousCommandDone = true;
		Robot.driveTrain.arcadeDrive(0, 0);
		Robot.isPositioning = false;
		doneAccelerating = false;
		currentMaxSpeed = 0;
		inVisionRange = false;
		Robot.limelightCamera.setLEDMode(LedMode.kforceOff);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	private void doAccel() {
		if (!doneAccelerating) {
			currentMaxSpeed = currentMaxSpeed + rampIncrement;
			if (currentMaxSpeed > mySpeed) {
				currentMaxSpeed = mySpeed;
				doneAccelerating = true;
			}
		}

	}
}
