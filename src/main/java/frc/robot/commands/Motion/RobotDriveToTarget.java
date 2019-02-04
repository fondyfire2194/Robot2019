package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;

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
	private double startingTargetAngle;

	public double myDistance;
	public double slowDownFeet = 2;
	public boolean decelerate;
	private double myEndpoint;
	private double startOfVisionPoint = 8;

	private double endOfVisionPoint = 1;
	private boolean inVisionRange;
	private double remainingFtToHatch;
	private double activeMotionComp;
	private double maxTargetArea = 20;

	// side distances are in inches
	// side speeds are in per unit where .25 = 25%
	// inPositionband is in feet

	public RobotDriveToTarget(double distance, double speed, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myDistance = distance;
		myTimeout = timeout;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		rampIncrement = mySpeed / 25;
		setTimeout(myTimeout);
		Robot.isPositioning = true;
		startingTargetAngle = Robot.driveTrain.getGyroYaw();
		currentMaxSpeed = 0;
		doneAccelerating = false;
		decelerate = false;
		slowDownFeet = Pref.getPref("DriveSldnDist");

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

		// in vision zone keep gyro target angle current in case need to switch
		// over to gyro

		inVisionRange = remainingFtToHatch < startOfVisionPoint && remainingFtToHatch > endOfVisionPoint;

		Robot.useVisionComp = inVisionRange && Robot.limelightCamera.getIsTargetFound();
		useGyroComp = !Robot.useVisionComp;

		if (Robot.useVisionComp) {
			Robot.activeMotionComp = Robot.limelightCamera.getdegRotationToTarget() * Pref.getPref("VisionKp");
			Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();
		}
		if (useGyroComp)
			activeMotionComp = Robot.driveTrain.getCurrentComp();

		Robot.driveTrain.arcadeDrive(currentMaxSpeed * Constants.FT_PER_SEC_TO_PCT_OUT, activeMotionComp);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.driveTrain.getLeftFeet() >= (myEndpoint)
				|| Robot.limelightCamera.getTargetArea() > Constants.MAX_TARGET_AREA;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		if (Robot.runningAutoCommand != 0)
			Robot.autonomousCommandDone[Robot.runningAutoCommand] = true;
		Robot.driveTrain.arcadeDrive(0, 0);
		Robot.isPositioning = false;
		doneAccelerating = false;
		decelerate = false;
		Robot.robotRotate.setSetpoint(startingTargetAngle);
		currentMaxSpeed = 0;
		inVisionRange = false;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
