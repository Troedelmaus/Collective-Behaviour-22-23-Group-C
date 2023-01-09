package org.vadere.simulator.models.ocean;

import org.jetbrains.annotations.NotNull;
import org.vadere.annotation.factories.models.ModelClass;
import org.vadere.simulator.models.MainModel;
import org.vadere.simulator.models.Model;
import org.vadere.simulator.models.ocean.behaviour.*;
import org.vadere.simulator.projects.Domain;
import org.vadere.state.attributes.Attributes;
import org.vadere.state.attributes.models.AttributesReynolds;
import org.vadere.state.attributes.scenario.AttributesAgent;
import org.vadere.state.psychology.cognition.UnsupportedSelfCategoryException;
import org.vadere.state.scenario.DynamicElement;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.scenario.Topography;
import org.vadere.state.scenario.Obstacle;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VShape;
import org.vadere.util.geometry.shapes.Vector2D;


import java.util.*;

@ModelClass(isMainModel = true)
public class OceanSteeringModel implements MainModel {

	private AttributesReynolds attributesReynolds;
	private AttributesAgent attributesPedestrian;
	private Random random;
	private Domain domain;

	private Perception bPerception;
	private Decision bDecision;

	private List<Model> submodels;

	public List<Obstacle> obst;

	public OceanSteeringModel() {

		//this.bDecision = new Decision(this);
	}

	@Override
	public void initialize(List<Attributes> modelAttributesList, Domain domain,
	                       AttributesAgent attributesPedestrian, Random random) {

		this.attributesReynolds = Model.findAttributes(modelAttributesList, AttributesReynolds.class);
		this.attributesPedestrian = attributesPedestrian;
		this.domain = domain;
		this.random = random;
		this.bPerception = new Perception(this, domain);
		this.bDecision = new Decision(this); // own logic
		this.obst = bPerception.getObstacles(); //own logic

		submodels = Collections.singletonList(this);

	}

	@Override
	public void preLoop(final double simTimeInSec) {}

	@Override
	public void postLoop(final double simTimeInSec) {}

	@Override
	public void update(final double simTimeInSec) {
		Collection<Pedestrian> pedestrians = domain.getTopography().getElements(Pedestrian.class);

		UnsupportedSelfCategoryException.throwIfPedestriansNotTargetOrientied(pedestrians, this.getClass());

		Iterator<Pedestrian> it = pedestrians.iterator();
		double maxSpeed = 3;


		while (it.hasNext()) {
			PedestrianOcean ped = (PedestrianOcean) it.next();
			Vector2D mov = new Vector2D(0, 0);

			//own logic
			List<Pedestrian> percievedPeds = bPerception.getPedInfo(ped);

			mov = mov.add(bDecision.seek(simTimeInSec, mov, ped));
			mov = mov.add(bDecision.nextStep(simTimeInSec, mov, ped, percievedPeds));
			mov = mov.add(bDecision.objectEvasion(simTimeInSec, mov, ped, obst));

			//bDecision mit bPerception füttern

			//end own logic

			// if movement is faster than max speed,
			// no normal movement is available, skip this turn.
			if (mov.getLength() > maxSpeed) {
				mov = new Vector2D(0, 0);
			}

			ped.move(simTimeInSec, mov);
		}
	}

	public Topography getScenario() {
		return this.domain.getTopography();
	}

	public AttributesAgent getAttributesPedestrian() {
		return this.attributesPedestrian;
	}

	public AttributesReynolds getAttributesReynolds() {
		return this.attributesReynolds;
	}

	@Override
	public <T extends DynamicElement> Pedestrian createElement(VPoint position, int id, Class<T> type) {
		return createElement(position, id, this.attributesPedestrian, type);
	}

	@Override
	public <T extends DynamicElement> Pedestrian createElement(VPoint position, int id, Attributes attr, Class<T> type) {
		AttributesAgent aAttr = (AttributesAgent)attr;

		if (!Pedestrian.class.isAssignableFrom(type))
			throw new IllegalArgumentException("RSM cannot initialize " + type.getCanonicalName());
		AttributesAgent pedAttributes = new AttributesAgent(
				aAttr, registerDynamicElementId(domain.getTopography(), id));
		Pedestrian result = create(position, pedAttributes);
		return result;
	}

	private Pedestrian create(@NotNull final VPoint position, @NotNull final AttributesAgent attributesAgent) {
		Pedestrian pedestrian = new PedestrianOcean(attributesAgent, random);
		pedestrian.setPosition(position);
		return pedestrian;
	}

	@Override
	public VShape getDynamicElementRequiredPlace(@NotNull final VPoint position) {
		return create(position, new AttributesAgent(attributesPedestrian, -1)).getShape();
	}

	@Override
	public List<Model> getSubmodels() {
		return submodels;
	}
}
