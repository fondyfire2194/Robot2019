package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.*;


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
public class ArmDriveToTarget extends Command {
	private double mySpeed;
	private boolean myEndItNow;
	private double myTimeout;
	private double rampIncrement;
	private boolean doneAccelerating;
	public static double currentMaxSpeed;
	public double slowDownCounts;
	public boolean decelerate;
	private double myEndpoint;
	private double remainingCounts;

	

	public ArmDriveToTarget(double distance, double speed, boolean endItNow, double timeout) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		

		myEndpoint = distance;
		mySpeed = speed;
		myEndItNow = endItNow;
		myTimeout = timeout;
	
	}

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
		rampIncrement = mySpeed / 25;
		setTimeout(myTimeout);
		currentMaxSpeed = 0;
		doneAccelerating = false;
		decelerate = false;
		slowDownCounts = Pref.getPref("ArmSldnDist");
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
		remainingCounts = myEndpoint - Robot.climber.getArmEncoderPosition();

		if (doneAccelerating && !decelerate && remainingCounts < slowDownCounts) {
			decelerate = true;
		}
		if (decelerate) {
			currentMaxSpeed = (mySpeed * remainingCounts) / slowDownCounts;
		}


		Robot.climber.runClimberArm(currentMaxSpeed);

	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		return isTimedOut() || myEndItNow || Robot.climber.getArmEncoderPosition() >= myEndpoint;
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {


		doneAccelerating = false;
		decelerate = false;
		currentMaxSpeed = 0;

	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
