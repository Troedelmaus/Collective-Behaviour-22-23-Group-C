package org.vadere.simulator.models.ocean.behaviour;

import org.vadere.simulator.models.ocean.PedestrianOcean;
import org.vadere.simulator.models.ocean.OceanSteeringModel;
import org.vadere.state.scenario.Obstacle;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.simulation.FootStep;
import org.vadere.state.simulation.FootstepHistory;
import org.vadere.util.geometry.shapes.Vector2D;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VLine;
import org.vadere.util.geometry.shapes.VCircle;
import org.vadere.util.geometry.shapes.VShape;
import org.vadere.util.geometry.GeometryUtils;



import java.util.*;

public class Decision {

    private OceanSteeringModel model;

    public Decision(OceanSteeringModel model) {
        this.model = model;
    }

    public Vector2D nextStep(double simTime, Vector2D currentMov, PedestrianOcean self, List<Pedestrian> percievedPeds) {
        //https://github.com/snape/RVO2-Java , https://gamma.cs.unc.edu/ORCA/publications/ORCA.pdf , https://www.mathworks.com/help/fuzzy/types-of-fuzzy-inference-systems.html
        double [] oceanValues = self.getOceanValues(); //NeighborDist; MaxNeighbors; TimeHorizon; Radius; PrefVelocity
        double radius = oceanValues[3];
        double speed;
        double neighToIntersec;
        double ownToIntersec;

        Collection<Pedestrian> peds = percievedPeds;
        Iterator<Pedestrian> it = peds.iterator();
        Pedestrian p;

        List<Vector2D> neighborTrajectory = new ArrayList<>();
        List<Vector2D> neighborPosition = new ArrayList<>();
        List<VPoint> intersections = new ArrayList<>();
        List<Pedestrian> intersectingPeds = new ArrayList<>();
        List<Pedestrian> relevantIntersectingPeds = new ArrayList<>();

        Vector2D toNeighbor;
        Vector2D proj;
        Vector2D norm;

        VPoint intersec;

        VLine ownLine;
        VLine neighLine;

        FootstepHistory footstepHistory;
        FootStep cfs;

        // Only avoid collisions, which would occur in front of us.
        Vector2D pos = new Vector2D(self.getPosition().add(currentMov.normalize(radius))); // vector in direction of seek with length radius
        Vector2D mov = currentMov.sub(currentMov.normalize(radius));

        while (it.hasNext()) {
            p = it.next();
            if (p.getId() == self.getId()) {
                continue;
            }
            // project toNeighbor to current movement
            toNeighbor = new Vector2D(p.getPosition().subtract(pos));
            proj = mov.multiply(
                    (mov.x * toNeighbor.x + mov.y * toNeighbor.y)
                            / (Math.pow(mov.x, 2) + Math.pow(mov.y, 2)));

            // if projection has different leading sign than current movement, our neighbor is
            // behind us.
            if ((proj.x < 0 && mov.x > 0 || proj.x > 0 && mov.x < 0) ||
                    (proj.y < 0 && mov.y > 0 || proj.y > 0 && mov.y < 0)) {

                continue;
            }
            // if projection is shorter than twice the ped radius, our neighbor is too
            // close, we have to skip our current movement
            if (proj.getLength() < radius) {
                return mov.multiply(-1);
            }
            footstepHistory = p.getFootstepHistory();
            if (footstepHistory.size() != 0) {
                /*double avgSpeed = footstepHistory.getAverageSpeedInMeterPerSecond();
                FootStep cfs = footstepHistory.getYoungestFootStep();
                neighborTrajectory.add((new Vector2D(cfs.getEnd().getX() - cfs.getStart().getX(), cfs.getEnd().getY() - cfs.getStart().getY())).normalize(avgSpeed * oceanValues[2]));*/
                //first uses avg Speed, second uses speed of last step
                cfs = footstepHistory.getYoungestFootStep();
                speed = cfs.length()/cfs.duration();
                neighborTrajectory.add((new Vector2D(cfs.getEnd().getX() - cfs.getStart().getX(), cfs.getEnd().getY() - cfs.getStart().getY())).normalize(speed * oceanValues[2]));
                neighborPosition.add(new Vector2D(p.getPosition()));
                intersectingPeds.add(p);
            }
        }
        if(neighborTrajectory.size() == 0){
            return new Vector2D(0, 0);
        }
        Iterator<Vector2D> itTrajectory = neighborTrajectory.iterator();
        Iterator<Vector2D> itPosition = neighborPosition.iterator();
        it = intersectingPeds.iterator();
        Vector2D neighTraj;
        Vector2D neighPos;

        Vector2D ownTraj = currentMov;
        Vector2D ownPos = new Vector2D(self.getPosition());


        while(itTrajectory.hasNext()) {
            neighTraj = itTrajectory.next();
            neighPos = itPosition.next();
            p = it.next();

            ownLine = new VLine(ownPos.x,ownPos.y,ownPos.x+ownTraj.x,ownPos.y+ownTraj.y);
            neighLine = new VLine(neighPos.x,neighPos.y,neighPos.x+neighTraj.x,neighPos.y+neighTraj.y);

            if(GeometryUtils.intersectLine(ownLine, new VPoint(neighPos.x,neighPos.y), new VPoint(neighPos.x+neighTraj.x,neighPos.y+neighTraj.y))){
                if(GeometryUtils.intersectLine(neighLine, new VPoint(ownPos.x,ownPos.y), new VPoint(ownPos.x+ownTraj.x,ownPos.y+ownTraj.y))) {
                    intersec = new VPoint(GeometryUtils.intersectionPoint(ownLine,neighLine));
                    neighToIntersec = intersec.distance(p.getPosition())/(neighTraj.getLength()/oceanValues[2]);
                    ownToIntersec = intersec.distance(self.getPosition())/oceanValues[4];
                    if(neighToIntersec-ownToIntersec < 0.5 && ownToIntersec-neighToIntersec < 0.5){
                        intersections.add(intersec);
                        relevantIntersectingPeds.add(p);
                    }
                }
            }
        }
        if(intersections.size() == 0){
            return new Vector2D(0, 0);
        }
        Iterator<VPoint> itInter = intersections.iterator();
        it = relevantIntersectingPeds.iterator();
        Pedestrian mostSignificantNeigh = it.next();
        double distIntersec = itInter.next().distance(self.getPosition());
        while(it.hasNext()) {
            p = it.next();
            intersec = itInter.next();
            if(distIntersec>intersec.distance(self.getPosition())){
                mostSignificantNeigh = p;
                distIntersec = intersec.distance(self.getPosition());
            }
        }
        cfs = mostSignificantNeigh.getFootstepHistory().getYoungestFootStep();
        mov = (new Vector2D(cfs.getEnd().getX() - cfs.getStart().getX(), cfs.getEnd().getY() - cfs.getStart().getY())).normalize(currentMov.getLength());
        // return difference between new and old movement as our correction vector
        return mov.sub(currentMov);
    }

    public Vector2D objectEvasion(double simTimeInSec, Vector2D currentMov, PedestrianOcean ped, List<Obstacle> obst){
        VPoint pos = ped.getPosition();
        Vector2D posVec = new Vector2D(ped.getPosition());
        VPoint posFut = new VPoint((posVec.add(currentMov)).x,(posVec.add(currentMov)).y);

        VShape closestIntersectingShape = null;
        double dist = 0;
        for (Obstacle obstacle : obst) {
            VLine pathLine = new  VLine(pos,new VPoint((posVec.add(currentMov.multiply(10))).x,(posVec.add(currentMov.multiply(10))).y));
            if(obstacle.getShape().intersects(pathLine)) {
                if (closestIntersectingShape == null) {
                    closestIntersectingShape = obstacle.getShape();
                    dist = obstacle.getShape().distance(pos);
                } else if (dist > obstacle.getShape().distance(pos)) {
                    closestIntersectingShape = obstacle.getShape();
                    dist = obstacle.getShape().distance(pos);
                }
            }
        }
        if(closestIntersectingShape == null){
            return new Vector2D(0, 0);
        }

        if(closestIntersectingShape.distance(pos)<(ped.getOceanValues())[4]){
            Vector2D mov = currentMov;
            if(pos.x>closestIntersectingShape.getCentroid().x){
                //ped is left of center
                while(true){
                    mov = mov.rotate(Math.PI*2 - Math.PI/20);
                    if(closestIntersectingShape.contains(new VPoint(mov.add(posVec).x,mov.add(posVec).y))){
                        continue;
                    } else {
                        return mov.sub(currentMov);
                    }
                }
            } else {
                while(true){
                    mov = mov.rotate(Math.PI/6);
                    if(closestIntersectingShape.contains(new VPoint(mov.add(posVec).x,mov.add(posVec).y))){
                        continue;
                    } else {
                        return mov.sub(currentMov);
                    }
                }
            }
        }
        return new Vector2D(0, 0);
    }

    public Vector2D seek(double simTime, Vector2D currentMov, PedestrianOcean ped) {
        double [] oceanValues = ped.getOceanValues();
        double maxSpeed = oceanValues[4];
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