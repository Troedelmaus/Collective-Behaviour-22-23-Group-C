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

	private Seek bSeek;
	private Separation bSeparation;
	private Containment bContainment;
	private CollisionAvoidance bCollisionAvoidance;
	private WallAvoidance bWallAvoidance;
	private Wander bWander;
	private List<Model> submodels;

	public OceanSteeringModel() {
		this.bSeek = new Seek(this);
		this.bSeparation = new Separation(this);
		this.bContainment = new Containment(this);
		this.bCollisionAvoidance = new CollisionAvoidance(this);
		this.bWallAvoidance = new WallAvoidance(this);
		this.bWander = new Wander(this);
	}

	@Override
	public void initialize(List<Attributes> modelAttributesList, Domain domain,
	                       AttributesAgent attributesPedestrian, Random random) {

		this.attributesReynolds = Model.findAttributes(modelAttributesList, AttributesReynolds.class);
		this.attributesPedestrian = attributesPedestrian;
		this.domain = domain;
		this.random = random;

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

			mov = mov.add(bSeek.nextStep(simTimeInSec, mov, ped));//aims and accelerates ped to destiny
			mov = mov.add(bWander.nextStep(simTimeInSec, mov, ped));//doesnt do anything
			mov = mov.add(bCollisionAvoidance.nextStep(simTimeInSec, mov, ped));
			mov = mov.add(bWallAvoidance.nextStep(simTimeInSec, mov, ped));//doesnt do anything
			mov = mov.add(bSeparation.nextStep(simTimeInSec, mov, ped));//moves in opposit direction of close pedestrians
			mov = mov.add(bContainment.nextStep(simTimeInSec, mov, ped));

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


	public double[] getNeighborDistMembership(double value){
		return trapezoid4(value,6.0,12.0,18.0,24.0);
	}
	public double[] getMaxNeighborsMembership(double value){
		return trapezoid4(value,10.0,20.0,30.0,40.0);
	}
	public double[] getTimeHorizonMembership(double value){
		return trapezoid4(value,5.0,10.0,10.0,15.0);
	}
	public double[] getRadiusMembership(double value){
		return trapezoid4(value,0.5,1.0,1.0,1.5);
	}
	public double[] getPrefVelocityMembership(double value){
		return trapezoid4(value,10.0,20.0,30.0,40.0);
	}


	public double[] trapezoid4(double value, double a, double b, double c, double d ){
		double[] memberships = {0.0,0.0,0.0};
		if(value <= a){
			memberships[0] = 1;//negative
			memberships[1] = 0;//neutral
			memberships[2] = 0;//positive
		} else if (value < b) {
			memberships[0] = (value*-1+b)/(b-a);//negative
			memberships[1] = (value-a)/(b-a);//neutral
			memberships[2] = 0;//positive
		} else if (value <= c) {
			memberships[0] = 0;//negative
			memberships[1] = 1;//neutral
			memberships[2] = 0;//positive
		} else if (value < d) {
			memberships[0] = 0;//negative
			memberships[1] = (value*-1+d)/(c-d);//neutral
			memberships[2] = (value-c)/(c-d);//positive
		} else {
			memberships[0] = 0;//negative
			memberships[1] = 0;//neutral
			memberships[2] = 1;//positive
		}
		return memberships;
	}


}
