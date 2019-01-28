/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants;
import frc.robot.Pref;
import frc.robot.LimeLight;

/**
 *
 * @author John
 */
public class RobotOrientToVision extends Command {
	private double mySpeed;
	private double myAngle;
	private double myTimeout;
	private int passCount;
	private boolean inPosition;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	private double rampIncrement;
	private double startTime;
	private double rampTime;
	private double angleError;
	

	public RobotOrientToVision(double angle, double speed, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.robotRotate);
		requires(Robot.driveTrain);


		mySpeed = speed;
		myAngle = angle;
		myTimeout = timeout;
	
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {

		rampIncrement = mySpeed / 10;
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
		double Kp = Pref.getPref("RotateKp");//Robot.prefs.getDouble("RotateKp",015);
		double Ki = Pref.getPref("RotateKi");//Robot.prefs.getDouble("RotateKi",0.0001);
		double Kd = Pref.getPref("RotateKd");//Robot.prefs.getDouble("RotateKd",01);
		double Kf = Pref.getPref("RotateKf");//Robot.prefs.getDouble("RotateKf",.8);

        Robot.robotRotate.setPIDF(Kp,0,Kd,Kf);
		Robot.robotRotate.setMaxOut(Constants.MINIMUM_START_PCT);
		Robot.robotRotate.setSetpoint(myAngle);
		Robot.robotRotate.enablePID();
		Robot.orientRunning = true;
		setTimeout(myTimeout);
		currentMaxSpeed = Constants.MINIMUM_START_PCT;
		passCount = 0;
		
		startTime = Timer.getFPGATimestamp();
		doneAccelerating = false;
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
				rampTime = Timer.getFPGATimestamp() - startTime;
				SmartDashboard.putNumber("Ramptime", rampTime);
			}
		}

		double currentPIDOut = Robot.robotRotate.loopOutput;
		if(Robot.limelightCamera.getIsTargetFound()){
			double visionGain = currentPIDOut/Math.abs(Robot.limelightCamera.getdegRotationToTarget());
			
		}
		if (passCount > 5 && Math.abs(Robot.robotRotate.getError()) < Pref.getPref("RotateIzone"));//Robot.prefs.getDouble("RotateIzone",2));
			Robot.robotRotate.getPIDController()
					.setI(Pref.getPref("RotateKi"));//Robot.prefs.getDouble("RotateKi",.005));

		
		inPosition = Robot.limelightCamera.getIsTargetFound()&&Math.abs(Robot.limelightCamera.getdegRotationToTarget())<1;
		
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return passCount > 15 && (isTimedOut() || inPosition);
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
		Robot.robotRotate.setMaxOut(Constants.MINIMUM_START_PCT);
		Robot.robotRotate.disable();
		Robot.orientRunning = false;

		Robot.driveTrain.leftDriveOut(0);
		Robot.driveTrain.rightDriveOut(0);

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}