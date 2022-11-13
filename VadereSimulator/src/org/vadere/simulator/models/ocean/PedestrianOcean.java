package org.vadere.simulator.models.ocean;

import org.vadere.state.attributes.scenario.AttributesAgent;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.simulation.FootStep;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class PedestrianOcean extends Pedestrian {

	private Vector2D lastMovement;
	private List<Double> oceanValues;

	private double startTime;
	private double lastSimTimeInSec;

	public PedestrianOcean(AttributesAgent attributesPedestrian, Random random) {
		super(attributesPedestrian, random);

		this.oceanValues = new ArrayList<Double>();
		this.lastMovement = new Vector2D(0, 0);
		this.startTime = -1;
		this.lastSimTimeInSec = -1;
		for(int i=0;i<5;i++){
			this.oceanValues.add((random.nextDouble()-0.5)*200);
		}
	}

	public VPoint getLastMovement() {
		return lastMovement;
	}
	public List<Double> getOceanValues(){
		return oceanValues;
	}

	public double getStartTime() {
		return startTime;
	}

	public void move(double simTime, Vector2D mov) {
		lastMovement = mov;

		if (startTime < 0) {
			startTime = simTime;
			lastSimTimeInSec = 0;
		}

		VPoint oldPosition = getPosition();
		VPoint newPosition = oldPosition.add(mov);
		setPosition(newPosition);

        // TODO: the first footstep starts at the wrong time!
		clearFootSteps();
		FootStep currentFootstep = new FootStep(oldPosition, newPosition, lastSimTimeInSec, simTime);
		getTrajectory().add(currentFootstep);
		getFootstepHistory().add(currentFootstep);

		lastSimTimeInSec = simTime;
	}

}
