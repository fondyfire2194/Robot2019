package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;
import frc.robot.LimelightControlMode.*;

/**
 * use a trajectory. Specify the trajectory segment where vision should take the
 * place of gyro
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
		Robot.isPositioning = true;
		currentMaxSpeed = 0;
		doneAccelerating = false;
		decelerate = false;
		slowDownFeet = Pref.getPref("DriveSldnDist");
		Robot.activeMotionComp = 0.;
		Robot.driveTrain.driveStraightAngle = myTargetAngle;
		targetWasSeen = false;
		targetNotSeenCtr = 0;
		Robot.noCameraTargetFound = false;
		visionTurnGain = Pref.getPref("VisionKp");
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

		if (targetNotSeenCtr > 10)
			Robot.noCameraTargetFound = true;
		if (Robot.limelightCamera.getIsTargetFound())
			targetWasSeen = true;

		Robot.useVisionComp = inVisionRange && Robot.limelightCamera.getIsTargetFound();
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

		Robot.autonomousCommandDone = true;
		Robot.driveTrain.arcadeDrive(0, 0);
		Robot.isPositioning = false;
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
