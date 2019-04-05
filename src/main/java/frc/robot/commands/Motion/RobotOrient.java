/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands.Motion;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.LimelightControlMode.LedMode;
import frc.robot.Constants;
import frc.robot.Pref;

/**
 *
 * @author John
 */
public class RobotOrient extends Command {
	private double mySpeed;
	private double myAngle;
	private double myTimeout;
	private int passCount;
	private boolean myAccuracy;
	private boolean inPosition;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;
	private double angleError;

	public RobotOrient(double angle, double speed, boolean accuracy, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.robotRotate);
		requires(Robot.driveTrain);

		mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
		myAccuracy = accuracy;
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		rampIncrement = mySpeed;
		/*
		 * error = command - actual. Needs adjusting if outside =+/-180
		 * 
		 * if answer is + 200 actually want to move -160 so subtract 360
		 * 
		 * if answer is - 200 actually want to move +160 so add 360
		 * 
		 * command = - 170, actual = 160 error = - 330 adjusted error 30 so error + 360
		 */
		angleError = myAngle - Robot.driveTrain.getGyroYaw();
		if (angleError > 180)
			angleError -= 360;
		if (angleError < -180)
			angleError = 360;

		// if (angleError > 0)
		double Kp = Pref.getPref("RotateKp");
		double Ki = Pref.getPref("RotateKi");
		double Kd = Pref.getPref("RotateKd");
		double Kf = Pref.getPref("RotateKf");

		Robot.robotRotate.setPIDF(Kp, 0, Kd, Kf);
		Robot.robotRotate.setMaxOut(Constants.MINIMUM_START_PCT);
		Robot.robotRotate.setSetpoint(myAngle);
		Robot.robotRotate.enablePID();
		Robot.orientRunning = true;
		setTimeout(myTimeout);
		currentMaxSpeed = Constants.MINIMUM_START_PCT;
		passCount = 0;
		doneAccelerating = false;
		if (Robot.autoRunning)
			Robot.limelightCamera.setLEDMode(LedMode.kforceOn);


		Robot.gph.retractPusher();
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
		passCount++;
		Robot.robotRotate.setMaxOut(currentMaxSpeed);
		if (!doneAccelerating) {
			currentMaxSpeed = currentMaxSpeed + rampIncrement;
			if (currentMaxSpeed >= mySpeed) {
				currentMaxSpeed = mySpeed;
				doneAccelerating = true;
			}
		}
		if (passCount > 5 && Robot.robotRotate.closeToPosition())
		Robot.robotRotate.getPIDController().setI(Pref.getPref("RotateKi"));

		if (myAccuracy)
			inPosition = Robot.robotRotate.inPosition();
		else
			inPosition = Robot.robotRotate.closeToPosition();
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return passCount > 15 && (isTimedOut() || inPosition);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {

		Robot.autonomousCommandDone = true;
		Robot.robotRotate.setMaxOut(Constants.MINIMUM_START_PCT);
		Robot.robotRotate.disable();
		Robot.orientRunning = false;

		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);
		Robot.driveTrain.resetEncoders();

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
