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
 * cargo ship. Other distance sensing such as ultrasound could be mounted next
 * to the camera. This would mean no more traveling wires in the elevator loop.
 * The hatch cover pusher cylinders could be left extended and sensed being
 * pushed back. This is short range and needs traveling wires.
 * 
 * 
 * 
 * 
 */
public class RobotDriveToTarget extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private boolean useGyroComp;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	public double myDistance;
	public double slowDownFeet;
	public boolean decelerate;
	private double myEndpoint;
	private double startOfVisionPoint = 8;
	private double endOfVisionPoint = 1;
	private boolean inVisionRange;
	private double remainingFtToHatch;
	private double myTargetAngle;
	private boolean targetWasSeen;
	private int targetNotSeenCtr;
	private double visionTurnGain;
	private double distanceErrorAtStart;
	private boolean useVisionComp;
	private double activeMotionComp;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in feet

	public RobotDriveToTarget(double distance, double speed, double targetAngle, boolean endItNow, double timeout) {
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
		Robot.positionRunning = true;
		currentMaxSpeed = 0;
		doneAccelerating = false;
		decelerate = false;
		slowDownFeet = Pref.getPref("DriveSldnDist");
		activeMotionComp = 0.;
		Robot.driveTrain.driveStraightAngle = myTargetAngle;
		targetWasSeen = false;
		targetNotSeenCtr = 0;
		Robot.noCameraTargetFound = false;
		distanceErrorAtStart = 0;
		visionTurnGain = Pref.getPref("VisionKp");
		if (Robot.limelightCamera.getIsTargetFound()) {
			distanceErrorAtStart = myDistance - Robot.visionData.getRobotVisionDistance();
			SD.putN2("DERAS", distanceErrorAtStart);
			if (Math.abs(distanceErrorAtStart) < 1.)
				myDistance = Robot.visionData.getRobotVisionDistance();
		}
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		if (!doneAccelerating) {
			currentMaxSpeed = currentMaxSpeed + rampIncrement;
			if (currentMaxSpeed > mySpeed) {
				currentMaxSpeed = mySpeed;
				doneAccelerating = true;
			}
		}
		remainingFtToHatch = myEndpoint - Robot.driveTrain.getLeftFeet();

		if (doneAccelerating && !decelerate && remainingFtToHatch < slowDownFeet) {
			decelerate = true;
		}
		if (decelerate) {
			currentMaxSpeed = (mySpeed * remainingFtToHatch) / slowDownFeet;
			if (currentMaxSpeed < .3)
				currentMaxSpeed = .3;
		}

		inVisionRange = (remainingFtToHatch < startOfVisionPoint && remainingFtToHatch > endOfVisionPoint)
				|| Robot.limelightCamera.getTargetArea() > Constants.MAX_TARGET_AREA;

		// in vision zone keep gyro target angle current in case need to switch
		// over to gyro

		if (inVisionRange)
			Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();

		if (inVisionRange & !targetWasSeen)
			targetNotSeenCtr++;

		// if (targetNotSeenCtr > 10)
		// Robot.noCameraTargetFound = true;
		if (Robot.limelightCamera.getIsTargetFound())
			targetWasSeen = true;

		useVisionComp = inVisionRange && Robot.limelightCamera.getIsTargetFound();
		useGyroComp = !useVisionComp;

		if (useVisionComp) {
			if (Robot.limelightOnEnd) {
				activeMotionComp = Robot.limelightCamera.getdegVerticalToTarget() * visionTurnGain;
			} else {
				activeMotionComp = Robot.limelightCamera.getdegRotationToTarget() * visionTurnGain;
			}

		}
		if (useGyroComp) {
			activeMotionComp = Robot.driveTrain.getCurrentComp();
		}
		Robot.driveTrain.arcadeDrive(currentMaxSpeed * Constants.FT_PER_SEC_TO_PCT_OUT, activeMotionComp);
		Robot.driveTrain.remainingDistance = remainingFtToHatch;
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
		Robot.positionRunning = false;
		doneAccelerating = false;
		decelerate = false;
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
}
