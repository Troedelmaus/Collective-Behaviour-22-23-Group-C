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
    private OceanSteeringModel model;


    public Perception(OceanSteeringModel model, Domain domain){
        this.model = model;
        this.topography = domain.getTopography();
    }

    public List<Pedestrian> getPedInfo(PedestrianOcean self){
        double [] oceanValues = self.getOceanValues(); //NeighborDist CHECK; MaxNeighbors CHECK; TimeHorizon ; Radius CHECK; PrefVelocity CHECK
        Collection<Pedestrian> peds = model.getScenario().getElements(Pedestrian.class);
        Iterator<Pedestrian> it = peds.iterator();
        Iterator<Pedestrian> itTmp;
        Vector2D pos = new Vector2D(self.getPosition());
        Pedestrian p;
        Pedestrian pp;
        ArrayList<Double> distToNeighbor = new ArrayList<>();
        List<Pedestrian> percievedPeds = new ArrayList<Pedestrian>();
        double max;
        while (it.hasNext()) {

            p = it.next();
            Vector2D pedPos = new Vector2D(p.getPosition());
            Vector2D toNeighbor = new Vector2D(p.getPosition().subtract(pos));
            Vector2D pedVel = new Vector2D(p.getVelocity());
            if (p.getId() == self.getId() || toNeighbor.getLength() <= oceanValues[0]) {
                continue;
            }
            percievedPeds.add(p);
            if(percievedPeds.size()>oceanValues[1]){
                itTmp = percievedPeds.iterator();
                distToNeighbor.removeAll(distToNeighbor);
                while (itTmp.hasNext()) {
                    pp = itTmp.next();
                    distToNeighbor.add((new Vector2D(pp.getPosition().subtract(pos))).getLength());
                }
                itTmp = percievedPeds.iterator();
                Collections.sort(distToNeighbor);
                max = distToNeighbor.get(distToNeighbor.size() - 1);
                while (itTmp.hasNext()){
                    pp = itTmp.next();
                    if((new Vector2D(pp.getPosition().subtract(pos))).getLength() == max){
                        percievedPeds.remove(pp);
                        break;
                    }
                }
            }
        }
        return percievedPeds;

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
}