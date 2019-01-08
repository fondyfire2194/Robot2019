package frc.robot;

import jaci.pathfinder.Trajectory;

/**
 * The DistanceFollower is an object designed to follow a trajectory based on
 * distance covered input. This class can be used for Tank or Swerve drive
 * implementations.
 *
 * @author Jaci
 */
public class ReverseDistanceFollower {

	double kp, ki, kd, kv, ka;

	double last_error, heading;
	double lastSegmentPosition;
	int segment;
	Trajectory trajectory;

	public ReverseDistanceFollower(Trajectory traj) {
		this.trajectory = traj;
	}

	public ReverseDistanceFollower() {
	}

	/**
	 * Set a new trajectory to follow, and reset the cumulative errors and segment
	 * counts
	 */
	public void setTrajectory(Trajectory traj) {
		this.trajectory = traj;
		reset();
		lastSegmentPosition = getLastSegmentPosition();
	}

	/**
	 * Configure the PID/VA Variables for the Follower
	 * 
	 * @param kp The proportional term. This is usually quite high (0.8 - 1.0 are
	 *           common values)
	 * @param ki The integral term. Currently unused.
	 * @param kd The derivative term. Adjust this if you are unhappy with the
	 *           tracking of the follower. 0.0 is the default
	 * @param kv The velocity ratio. This should be 1 over your maximum velocity @
	 *           100% throttle. This converts m/s given by the algorithm to a scale
	 *           of -1..1 to be used by your motor controllers
	 * @param ka The acceleration term. Adjust this if you want to reach higher or
	 *           lower speeds faster. 0.0 is the default
	 */
	public void configurePIDVA(double kp, double ki, double kd, double kv, double ka) {
		this.kp = kp;
		this.ki = ki;
		this.kd = kd;
		this.kv = kv;
		this.ka = ka;
	}

	/**
	 * Reset the follower to start again. Encoders must be reconfigured.
	 */
	public void reset() {
		last_error = 0;
		segment = trajectory.length() - 1;

	}

	/**
	 * Calculate the desired output for the motors, based on the distance the robot
	 * has covered. This does not account for heading of the robot. To account for
	 * heading, add some extra terms in your control loop for realignment based on
	 * gyroscope input and the desired heading given by this object.
	 * 
	 * @param distance_covered The distance covered in feet
	 * @return The desired output for your motor controller
	 */
	// public double calculate(double distance_covered) {
	// if (segment < trajectory.length()) {
	// Trajectory.Segment seg = trajectory.get(segment);
	// double error = seg.position - distance_covered;
	// double calculated_value = kp * error + // Proportional
	// kd * ((error - last_error) / seg.dt) + // Derivative
	// (kv * seg.velocity + ka * seg.acceleration); // V and A Terms
	// last_error = error;
	// heading = seg.heading;
	// segment++;
	//
	// return calculated_value;
	// } else
	// return 0;
	// }

	public double getLastSegmentPosition() {
		Trajectory.Segment seg = trajectory.get(trajectory.length() - 1);
		return seg.position;
	}

	/*
	 * Trying to run a trajectory starting at last position and running down to
	 * first.
	 * 
	 * Subtracting each segment position from the last segment position will give an
	 * increasing position value from 0 that is needed by the trajctory code.
	 * 
	 * For example if original positions were 0,1,3,6,9 then the last value is 9 and
	 * differences and new values are 9,8,5,3,0
	 * 
	 * Also accelerations become decels and vice versa so their sign must be
	 * reversed
	 * 
	 * Segment number is set to trajectory length - and decremented to 0 as
	 * trajectory runs
	 * 
	 */
	public double calculate(double distance_covered) {
		if (segment > -1) {
			Trajectory.Segment seg = trajectory.get(segment);
			double error = lastSegmentPosition - seg.position - distance_covered;
			double calculated_value = kp * error + // Proportional
					kd * ((error - last_error) / seg.dt) + // Derivative
					(kv * seg.velocity + ka * -seg.acceleration); // V and A Terms
			last_error = error;
			heading = seg.heading;
			segment--;
			return calculated_value;
		} else
			return 0;
	}

	/**
	 * @return the desired heading of the current point in the trajectory
	 */
	public double getHeading() {
		return heading;
	}

	/**
	 * @return the current segment being operated on
	 */
	public Trajectory.Segment getSegment() {
		return trajectory.get(segment);
	}

	/**
	 * @return whether we have finished tracking this trajectory or not.
	 */
	public boolean isFinished() {
		return segment < 0;
	}

}
