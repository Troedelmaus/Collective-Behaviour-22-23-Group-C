package org.vadere.simulator.models.ocean.behaviour;

import org.vadere.simulator.models.ocean.PedestrianOcean;
import org.vadere.simulator.models.ocean.OceanSteeringModel;
import org.vadere.state.scenario.Obstacle;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.scenario.Topography;
import org.vadere.util.geometry.shapes.Vector2D;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VShape;
import org.vadere.util.geometry.shapes.IPoint;
import org.vadere.simulator.projects.Domain;


import java.util.*;

public class Perception {


    private final transient Topography topography;
    private Collection<Pedestrian> peds;
    private OceanSteeringModel model;
    private Domain domain;

    public Perception(OceanSteeringModel model, Domain domain){
        this.model = model;
        this.topography = domain.getTopography();
        this.peds = peds;
    }

    public ArrayList<ArrayList<Vector2D>> getPedInfo(PedestrianOcean self) {
        //returns a 2D Array of 2DVectors: 1. vector of position of neighbor,
        //                                 2. vector from self to neighbor (self+2=1)
        //                                 3. velocity of neighbor
        peds = model.getScenario().getElements(Pedestrian.class);
        Iterator<Pedestrian> it = peds.iterator();

        ArrayList<ArrayList<Vector2D>> pedInfo = new ArrayList<>();
        Vector2D pos = new Vector2D(self.getPosition());
        Pedestrian p;
        while (it.hasNext()) {

            p = it.next();
            if (p.getId() == self.getId()) {
                continue;
            }
            Vector2D pedPos = new Vector2D(p.getPosition());
            Vector2D toNeighbor = new Vector2D(p.getPosition().subtract(pos));
            Vector2D pedVel = new Vector2D(p.getVelocity());
            ArrayList<Vector2D> tmpList = new ArrayList<Vector2D>();
            tmpList.add(pedPos);
            tmpList.add(toNeighbor);
            tmpList.add(pedVel);
            pedInfo.add(tmpList);
        }
        return pedInfo;
    }
    public List<Obstacle> getObstacles() {
        //returns a List of all Obstacles
        Collection<Obstacle> obstacles = topography.getObstacles();
        List<Obstacle> result = new LinkedList<>();
        for (Obstacle obstacle : obstacles) {
            result.add(obstacle);
        }
        return result;
    }
    public ArrayList<Double> getObstacleDists(PedestrianOcean self){
        //returns a List of all distances between objects and one ped
        Vector2D pos = new Vector2D(self.getPosition());
        Collection<Obstacle> obstacles = topography.getObstacles();
        ArrayList<Double> result = new ArrayList<>();
        for (Obstacle obstacle : obstacles) {
            double tmpDist = obstacle.getShape().distance(pos);
            result.add(tmpDist);
        }
        return result;
    }
}