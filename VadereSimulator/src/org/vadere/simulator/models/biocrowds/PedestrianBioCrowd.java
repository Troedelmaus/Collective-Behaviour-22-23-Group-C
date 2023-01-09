package org.vadere.simulator.models.biocrowds;

import org.vadere.state.attributes.scenario.AttributesAgent;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.scenario.Topography;
import org.vadere.state.simulation.FootStep;
import org.vadere.util.geometry.shapes.IPoint;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.ArrayList;
import java.util.Random;

public class PedestrianBioCrowd extends Pedestrian {

    private final Topography topography;
    private final Integer size;

    private final ArrayList<Vector2D> markers;
    private final ArrayList<Double> weights;
    private final double max_speed;

    public boolean isDone() {
        return done;
    }

    private boolean done;
    private double startTime;
    private Vector2D lastMovement;
    private double lastSimTimeInSec;
    private VPoint target;
    private BioCrowd model;

    public PedestrianBioCrowd(VPoint position, Topography topography, AttributesAgent attributesAgent, Random random, VPoint target) {
        super(attributesAgent, random);
        super.setPosition(position);

        this.topography = topography;
        this.startTime = -1;
//        super.setTargets(target);
        this.size = 2;
        this.markers = new ArrayList<Vector2D>();
        this.weights = new ArrayList<Double>();
        this.max_speed = 0.05;
        this.done = false;
        this.lastMovement = new Vector2D(0, 0);
        this.lastSimTimeInSec = -1;
        this.target = target;

    }

    public double getStartTime() {
        return startTime;
    }

    public ArrayList<Vector2D> getMarkers() {
        return markers;
    }

    public void addMarker(Vector2D marker){
        markers.add(marker);
    }

    public void clearMarkers(){
        markers.clear();
    }

    public void move(double simTime, Vector2D mov) {

//        if (this.done) return;

//        VPoint pos = getPosition();
//        VPoint target = model.getScenario().getTarget(getTargets().getFirst()).getShape().closestPoint(pos);

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

        VPoint pos = getPosition();
        Vector2D toTarget = new Vector2D(target.subtract(pos));

        double distToTarget = toTarget.getLength();
//        if (distToTarget < 0.5){
//            this.done = true;
//        }
    }
}
