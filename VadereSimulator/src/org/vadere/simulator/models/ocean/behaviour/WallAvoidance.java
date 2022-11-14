package org.vadere.simulator.models.ocean.behaviour;

import org.vadere.simulator.models.ocean.OceanSteeringModel;
import org.vadere.simulator.models.ocean.PedestrianOcean;
import org.vadere.util.geometry.shapes.Vector2D;
import org.vadere.state.scenario.Obstacle;

import java.util.Collection;
import java.util.Iterator;

public class WallAvoidance {

	private OceanSteeringModel model;

	public WallAvoidance(OceanSteeringModel model) {
		this.model = model;
	}

	public Vector2D nextStep(double simTime, Vector2D currentMov, PedestrianOcean self) {

		//double radius = self.getAttributes().getRadius();
		//Collection<Obstacle> walls = model.getScenario().getObstacles();
		//Iterator<Obstacle> it = walls.iterator();
		//while(it.hasNext()){
		//
		//	.shape[[60.0001,20.0001],[-1.0E-4,20.0001],[-1.0E-4,19.4999],[60.0001,19.4999]]
		//}
		Vector2D mov = new Vector2D(0, 0);
		Vector2D nextPos = new Vector2D(self.getPosition()).add(currentMov);

		closestObstacle = detectObstacleProximity(nextPos, 5);
		if(closestObstacle.isEmpty()){
			return mov;
		} else{
			//closestObstacle[0]
			//TODO Evade Object... how to get VPoint to calc with & how to determine which way is shorter
			
		}
		return mov;
	}
	public List<Obstacle> detectObstacleProximity(@NotNull VPoint position, double proximity) {

		Collection<Obstacle> obstacles = topography.getObstacles();
		double closest = proximity
		List<Obstacle> result = new LinkedList<>();
		for (Obstacle obstacle : obstacles) {
			if (obstacle.getShape().distance(position) < proximity && obstacle.getShape().distance(position)<closest) {
				List<Obstacle> result = new LinkedList<>();
				result.add(obstacle);
				closest = obstacle.getShape().distance(position)
			}
		}

		return result;
	}
}
