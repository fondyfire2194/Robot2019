package frc.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import frc.robot.Constants;
import frc.robot.Robot;

public class GeneratePositionTrajectory {

	public GeneratePositionTrajectory() {

	}

	// Prepare the Trajectory for Generation.
	//
	// Arguments:
	// Fit Function: FIT_HERMITE_CUBIC or FIT_HERMITE_QUINTIC
	// Sample Count: PATHFINDER_SAMPLES_HIGH (100 000)
	// PATHFINDER_SAMPLES_LOW (10 000)
	// PATHFINDER_SAMPLES_FAST (1 000)
	// Time Step: 0.05 Seconds
	// Max Velocity: 6 ft/s
	// Max Acceleration: 12 ft/s/s
	// Max Jerk: 10 ft/s/s/s

	public Trajectory[] generate(double endX, double endY, double angle) {
		Trajectory active[] = new Trajectory[2];
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, 6, 12.0, 10.0);

		Waypoint[] points = new Waypoint[] {

				// new Waypoint(3, 4, Pathfinder.d2r(0)),

				new Waypoint(0, 0, Pathfinder.d2r(0)),

				new Waypoint(endX, endY, Pathfinder.d2r(angle)), };

		Trajectory trajectory = Pathfinder.generate(points, config);

		// Wheelbase measure by rotating 10 times and calculating from encoder numbers

		TankModifier modifier = new TankModifier(trajectory).modify(Constants.WHEELBASE_WIDTH);

		// Do something with the new Trajectories...
		active[0] = modifier.getLeftTrajectory();

		active[1] = modifier.getRightTrajectory();

		return active;
	}

}
