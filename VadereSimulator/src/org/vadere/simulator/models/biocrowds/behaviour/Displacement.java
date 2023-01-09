package org.vadere.simulator.models.biocrowds.behaviour;

import org.vadere.simulator.models.biocrowds.BioCrowd;
import org.vadere.simulator.models.biocrowds.PedestrianBioCrowd;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.*;

public class Displacement {

	private BioCrowd model;

	public Displacement(BioCrowd model) {
		this.model = model;
	}

	public Vector2D nextStep(double simTime, Vector2D currentMov, PedestrianBioCrowd ped) {
//		double maxSpeed = ped.getAttributes().getSpeedDistributionMean();
		double maxSpeed = 1.0;
		double radius = 2.0;
		ArrayList<Vector2D> markers = ped.getMarkers();

		VPoint pos = ped.getPosition();
		Vector2D pos_vec = new Vector2D(pos.getX(), pos.getY());
		VPoint target = model.getScenario().getTarget(ped.getTargets().getFirst()).getShape().closestPoint(pos);
		Vector2D toTarget = new Vector2D(target.subtract(pos));
		Vector2D mov = toTarget.clone();

		for (Iterator<Vector2D> it = markers.iterator(); it.hasNext();){
			Vector2D marker = it.next();
			if (marker.getY() <= pos.getY()){
				it.remove();
			}
		}

		if (markers.isEmpty()){
			markers.add(new Vector2D(target.getX(), target.getY()));
		}

		ArrayList<Double> weights = computeMarkerWeights(markers, ped);

		Vector2D motionVector = new Vector2D(0, 0);


		for (int i = 0; i < weights.size(); i++){
			Vector2D dist = markers.get(i).clone();
			dist = dist.sub(pos);
			motionVector = motionVector.add(dist.multiply(weights.get(i)));
		}

		final double speed = Math.min(motionVector.getLength(), maxSpeed);

		final Vector2D disp = motionVector.multiply(1/motionVector.getLength()).multiply(speed);

		return disp;
	}

	private ArrayList<Double> computeMarkerWeights(Collection<Vector2D> markers, PedestrianBioCrowd ped){

		Collection<Double> f_weights = new java.util.ArrayList<>(Collections.emptyList());
		ArrayList<Double> weights = new java.util.ArrayList<>(Collections.emptyList());

		double total = 0.0;
		Iterator<Vector2D> it = markers.iterator();

		VPoint pos = ped.getPosition();
		VPoint target = model.getScenario().getTarget(ped.getTargets().getFirst()).getShape().closestPoint(pos);
		Vector2D toTarget = new Vector2D(target.subtract(pos));
		Vector2D v1 = toTarget.clone();

		while (it.hasNext()) {
			Vector2D v2 = it.next().clone();
			v1 = v1.sub(pos);
			v2 = v2.sub(pos);
			double f_weight = v1.dotProduct(v2);
			f_weights.add(f_weight);
			total += f_weight;
		}

		final double finalTotal = total;
		for (Double weight : f_weights) {
			if (finalTotal == 0.0){
				weights.add(0.0);
			}
			else weights.add(weight / finalTotal);
		}

		return weights;
	}
}
