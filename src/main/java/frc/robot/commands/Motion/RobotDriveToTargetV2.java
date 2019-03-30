package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import frc.robot.*;
import frc.robot.LimelightControlMode.*;
import frc.robot.LimeLight;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.MovingAverage;

/**
 * This command is used from a middle start where the distance and angle is
 * known precisely, but also following a rotate which followed a trajectory. In
 * this latter case, distance to the target is subject to variaton of possibly
 * as much as +/- 1/2 foot ? The distance can be up to 11 feet ? when
 * approaching the load station from Cargo Ship side, and 10 ft from cargo ship
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
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	public double myDistance;
	private double myEndpoint;
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
	private boolean correctionMade;
	private boolean useVisionComp;
	private boolean gyroLocked;
	private MovingAverage movingAverage = new MovingAverage(10);

	/**
	 * kp equivalent is the speed / slowdown feet or 7.5 ft/sec/2.5ft from previous
	 * testing = 3 So at 2 ft speed is 6 ft per sec, at 1 ft it is 3
	 */

	public RobotDriveToTargetV2(double distance, double speed, double targetAngle, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain);
		// movingAverage = new MovingAverage(10);
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
		Shuffleboard.startRecording();
		Robot.driveTrain.resetEncoders();
		rampIncrement = mySpeed / 25;
		setTimeout(myTimeout);
		Robot.positionRunning = true;
		currentMaxSpeed = Constants.MINIMUM_START_PCT * Constants.MAX_ROBOT_FT_PER_SEC;
		doneAccelerating = false;
		Robot.driveTrain.activeMotionComp = 0.;
		Robot.driveTrain.driveStraightAngle = myTargetAngle;
		targetWasSeen = false;
		targetNotSeenCtr = 0;
		Robot.noCameraTargetFound = false;
		lastRemainingDistance = 0;
		// "linear" vision range in ft
		visionTurnGain = Pref.getPref("VisionKp");
		Kp = Pref.getPref("DrivePositionKp");
		Kd = Pref.getPref("DrivePositionKd");
		correctionMade = false;
		Robot.limelightCamera.setSnapshot(Snapshot.kon);
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {

		/**
		 * capture data for this cycle
		 * 
		 */
		visionTargetSeen = Robot.limelightCamera.getIsTargetFound();
		robotDistance = Robot.driveTrain.getLeftFeet();
		remainingFtToHatch = myEndpoint - robotDistance;

		// check if camera can be used

		if (!doneAccelerating) {
			doAccel();
		}
		// if no target seen abort auto
		if (remainingFtToHatch < 6 & !targetWasSeen){
			doTargetSeenCheck();

		}
		if (Math.abs(getFilteredDegRotToTarget() - Robot.limelightCamera.getdegRotationToTarget()) < 1) {
			// vision and gyro comps
			doComps();
		} else {
			Shuffleboard.addEventMarker("GLITCH", EventImportance.kHigh);
		}
		// one time correcton of final distance from ultrasound
		if (Robot.useUltrasound && !correctionMade)
			doCorrection();
		// control speed of motion using kp and kd
		if (doneAccelerating)
			doSpeed();

		Robot.driveTrain.arcadeDrive(currentMaxSpeed * Constants.FT_PER_SEC_TO_PCT_OUT,
				Robot.driveTrain.activeMotionComp);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.driveTrain.getLeftFeet() >= myEndpoint - 1;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {

		Robot.driveTrain.arcadeDrive(0, 0);
		if (myEndItNow) {
			Robot.driveTrain.setLeftSideDriveBrakeOn(true);
			Robot.driveTrain.setRightSideDriveBrakeOn(true);
		}
		Shuffleboard.stopRecording();
		Robot.limelightCamera.setSnapshot(Snapshot.koff);
		Robot.positionRunning = false;
		doneAccelerating = false;
		currentMaxSpeed = 0;
		Robot.limelightCamera.setLEDMode(LedMode.kforceOff);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}

	private void doAccel() {

		currentMaxSpeed = currentMaxSpeed + rampIncrement;

		if (currentMaxSpeed > mySpeed) {
			currentMaxSpeed = mySpeed;
			doneAccelerating = true;
		}
	}

	private void doSpeed() {
		// differential
		positionChange = lastRemainingDistance - remainingFtToHatch;
		lastRemainingDistance = robotDistance;
		positionRateOfChange = positionChange / loopTime;

		calcLoopSpeed = remainingFtToHatch * Kp;// + positionRateOfChange * Kd;
		SmartDashboard.putNumber("PRFT", remainingFtToHatch);
		SmartDashboard.putNumber("PCLS", calcLoopSpeed);
		if (calcLoopSpeed > mySpeed)
			currentMaxSpeed = mySpeed;
		else
			currentMaxSpeed = calcLoopSpeed;

		// set minimum speed
		// if (currentMaxSpeed < 1.5) {
		// currentMaxSpeed = 1.5;
		// }
	}

	private void doTargetSeenCheck() {
		targetNotSeenCtr++;
		if (targetNotSeenCtr > 10)
			Robot.noCameraTargetFound = true;
	}

	private void doComps() {
		// in vision zone keep gyro target angle current in case need to switch
		// over to gyro

		if (visionTargetSeen) {
			useVisionComp = true;
			targetWasSeen = true;
		}
		if (useVisionComp && remainingFtToHatch < 6 && Math.abs(Robot.limelightCamera.getdegVerticalToTarget()) < 1.) {
			Robot.driveTrain.driveStraightAngle = Robot.driveTrain.getGyroYaw();
			// useVisionComp = false;
			SmartDashboard.putNumber("VWLI", Math.abs(Robot.limelightCamera.getdegVerticalToTarget()));

		}

		Robot.driveTrain.useGyroComp = !useVisionComp;

		if (useVisionComp) {
			if (Robot.limelightOnEnd) {
				Robot.driveTrain.activeMotionComp = (Robot.limelightCamera.getdegVerticalToTarget()
						+ Pref.getPref("DriveSldnDist")) * visionTurnGain;
			} else {
				Robot.driveTrain.activeMotionComp = Robot.limelightCamera.getdegRotationToTarget() * visionTurnGain;
			}
		}
		if (Robot.driveTrain.useGyroComp) {
			Robot.driveTrain.activeMotionComp = Robot.driveTrain.getCurrentComp();
		}

	}

	private void doCorrection() {
		if (remainingFtToHatch < 8) {
			double distanceDifference = Robot.ultrasound.getDistanceFeet() - remainingFtToHatch;
			if (Math.abs(distanceDifference) < Constants.USND_CORRECT_BAND)
				myEndpoint = myEndpoint + distanceDifference;
			correctionMade = true;
		}

	}

	public double getFilteredDegRotToTarget() {
		movingAverage.add(Robot.limelightCamera.getdegRotationToTarget());
		return movingAverage.getAverage();
	}

}
