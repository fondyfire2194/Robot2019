package frc.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;

public class GeneratePositionTrajectory {

	public GeneratePositionTrajectory() {

	}

	public Trajectory[] generate(double endX, double endY, double angle) {
		Trajectory buffer[] = new Trajectory[2];
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC,
				Trajectory.Config.SAMPLES_HIGH, 0.05, 6, 12.0, 10.0);

		Waypoint[] points = new Waypoint[] {

				new Waypoint(0, 0, Pathfinder.d2r(angle)),

				new Waypoint(endX, endY, Pathfinder.d2r(angle)), };

		Trajectory trajectory = Pathfinder.generate(points, config);

		// Wheelbase Width = 2.17 ft

		TankModifier modifier = new TankModifier(trajectory).modify(2.17);

		// Do something with the new Trajectories...
		buffer[0] = modifier.getLeftTrajectory();

		buffer[1] = modifier.getRightTrajectory();

		return buffer;
	}

}
