package org.vadere.simulator.models.biocrowds.behaviour;

import org.vadere.simulator.models.biocrowds.BioCrowd;
import org.vadere.simulator.models.biocrowds.PedestrianBioCrowd;
import org.vadere.simulator.models.reynolds.PedestrianReynolds;
import org.vadere.simulator.models.reynolds.ReynoldsSteeringModel;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

public class Seek {

	private BioCrowd model;

	public Seek(BioCrowd model) {
		this.model = model;
	}

	public Vector2D nextStep(double simTime, Vector2D currentMov, PedestrianBioCrowd ped) {
		double maxSpeed = ped.getAttributes().getSpeedDistributionMean();
		double simTimeStepLength = 0.4; // TODO [priority=low] [task=refactoring] get this attribute from AttributePedestrians

		VPoint pos = ped.getPosition();
		// VPoint target = model.getScenario().getTargets().get(0).getShape().closestPoint(pos);
		VPoint target = model.getScenario().getTarget(ped.getTargets().getFirst()).getShape().closestPoint(pos);
		Vector2D toTarget = new Vector2D(target.subtract(pos));
		Vector2D mov = toTarget.clone();

		// Cap to max speed
		mov = mov.normalize(maxSpeed);

		// Accelerate
		double startTime = ped.getStartTime();
		double pastTime = startTime < 0 ? 0 : simTime - ped.getStartTime();
		if (pastTime < simTimeStepLength + simTimeStepLength * 0.1) {
			mov.normalize(maxSpeed / 2);
		}

		// Arrive
		double stepsUntilTarget = 3;
		double distToTarget = toTarget.getLength();
		double slowed;
		if (distToTarget < maxSpeed * stepsUntilTarget) {
			slowed = distToTarget / stepsUntilTarget + maxSpeed * 0.1;
			if (mov.getLength() > slowed) {
				mov = mov.normalize(slowed);
			}
		}

		return mov;
	}

}
