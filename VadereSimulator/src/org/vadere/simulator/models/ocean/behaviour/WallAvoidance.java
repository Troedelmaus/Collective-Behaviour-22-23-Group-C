package org.vadere.simulator.models.ocean.behaviour;

import org.vadere.simulator.models.ocean.OceanSteeringModel;
import org.vadere.simulator.models.ocean.PedestrianOcean;
import org.vadere.util.geometry.shapes.Vector2D;

public class WallAvoidance {

	private OceanSteeringModel model;

	public WallAvoidance(OceanSteeringModel model) {
		this.model = model;
	}

	public Vector2D nextStep(double simTime, Vector2D currentMov, PedestrianOcean ped) {
		return new Vector2D(0, 0);
	}

}
