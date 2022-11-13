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

		double radius = self.getAttributes().getRadius();
		Collection<Obstacle> walls = model.getScenario().getObstacles();
		Iterator<Obstacle> it = walls.iterator();
		//while(it.hasNext()){

			//.shape[[60.0001,20.0001],[-1.0E-4,20.0001],[-1.0E-4,19.4999],[60.0001,19.4999]]
		//}
		return new Vector2D(0, 0);
	}

}
