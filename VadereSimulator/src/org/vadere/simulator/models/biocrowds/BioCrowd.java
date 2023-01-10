package org.vadere.simulator.models.biocrowds;

import org.jetbrains.annotations.NotNull;
import org.vadere.annotation.factories.models.ModelClass;
import org.vadere.simulator.models.MainModel;
import org.vadere.simulator.models.Model;
import org.vadere.simulator.models.biocrowds.behaviour.Displacement;
import org.vadere.simulator.models.biocrowds.behaviour.Seek;
import org.vadere.simulator.projects.Domain;
import org.vadere.state.attributes.Attributes;
import org.vadere.state.attributes.scenario.AttributesAgent;
import org.vadere.state.psychology.cognition.UnsupportedSelfCategoryException;
import org.vadere.state.scenario.DynamicElement;
import org.vadere.state.scenario.Obstacle;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.scenario.Topography;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VShape;
import org.vadere.util.geometry.shapes.Vector2D;
import org.vadere.util.geometry.shapes.VLine;

import java.util.*;

@ModelClass(isMainModel = true)
public class BioCrowd implements MainModel {
//    private final Integer grid_size;
//    private final Integer num_markers;
    private ArrayList<Vector2D> markers;
    private final ArrayList agents;
    private ArrayList<ArrayList<Object>> container;
    private AttributesBioCrowd attributesBioCrowd;
    private AttributesAgent attributesPedestrian;
    private Domain domain;
    private Random random;
    private List<Model> submodels;
    private Seek bSeek;
    private Displacement displacement;
    private VPoint target;

    private List<Obstacle> obstacles;

    public BioCrowd() {
//        this.markers = new ArrayList<Vector2D>();
        this.agents = new ArrayList();
        this.container = new ArrayList<>();
        this.bSeek = new Seek(this);
        this.displacement = new Displacement(this);
        this.obstacles = new ArrayList<>();

//        ScatterMarkers();
    }

    private void ScatterMarkers() {
        for (int i = 0; i < 100; i++){
            this.markers.add(new Vector2D(Math.random()*10, Math.random()*10));
        }
    }

    public Topography getScenario() {
        return this.domain.getTopography();
    }

    public AttributesAgent getAttributesPedestrian() {
        return this.attributesPedestrian;
    }

    public AttributesBioCrowd getAttributesBioCrowd() {
        return this.attributesBioCrowd;
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
        Pedestrian pedestrian = new PedestrianBioCrowd(position, null, attributesAgent, random, target);
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

    @Override
    public void initialize(List<Attributes> attributesList, Domain domain, AttributesAgent attributesPedestrian, Random random) {
        this.attributesBioCrowd = Model.findAttributes(attributesList, AttributesBioCrowd.class);
        this.attributesPedestrian = attributesPedestrian;
        this.domain = domain;
        this.random = random;
        // markers are initialized here
        this.markers = this.attributesBioCrowd.getMarkers(domain.getTopography().getBounds().width,domain.getTopography().getBounds().height);
        this.target = domain.getTopography().getTargetShapes().values().iterator().next().get(0).getCentroid();
        this.obstacles = getObstacles();

        submodels = Collections.singletonList(this);
    }

    @Override
    public void preLoop(double simTimeInSec) {

    }

    @Override
    public void postLoop(double simTimeInSec) {

    }

    @Override
    public void update(double simTimeInSec) {
        //get current pedestrians on the board
        Collection<Pedestrian> current_pedestrians = domain.getTopography().getElements(Pedestrian.class);

        UnsupportedSelfCategoryException.throwIfPedestriansNotTargetOrientied(current_pedestrians, this.getClass());

        Iterator<Pedestrian> it = current_pedestrians.iterator();
        double maxSpeed = 0.5;

//        ArrayList<Vector2D> taken_markers = new ArrayList<Vector2D>();

        //handle the first pedestrian
        handle_first_pedestrian(it);

        for (; it.hasNext();) {
            // converting a pedestrian into BioCrowd pedestrian
            PedestrianBioCrowd ped = (PedestrianBioCrowd) it.next();
            // creating a vector for movement
            Vector2D mov = new Vector2D(0, 0);

            //getting a current position of pedestrian
            VPoint pos = ped.getPosition();

            //getting a closest target of pedestrian
            VPoint target = getScenario().getTarget(ped.getTargets().getFirst()).getShape().closestPoint(pos);
            //a vector in a direction of target from pedestrian
            Vector2D toTarget = new Vector2D(target.subtract(pos));

            mov = mov.add(displacement.nextStep(simTimeInSec, mov, ped));
            mov = mov.add(avoid(mov,ped,obstacles));
            //move a pedestrian
            ped.move(simTimeInSec, mov);

            //return a markers from a prevous position
            markers.addAll(ped.getMarkers());
            // clearing those markers
            ped.clearMarkers();

            // getting a new postion
            VPoint new_pos = ped.getPosition();

            //iterate over markers in order to assign markers to a pedestrian and removes them from
            //markers, in order other pedestrians not to see them
            for (Iterator<Vector2D> it1 = markers.iterator(); it1.hasNext();){
                Vector2D marker = it1.next();
                double radius = 2.0;
                if (marker.distance(new_pos) < radius){
                    ped.addMarker(marker);
                    it1.remove();
                }
            }
        }

//        markers.addAll(taken_markers);

    }

    private void handle_first_pedestrian(Iterator<Pedestrian> it) {
        PedestrianBioCrowd first_ped = (PedestrianBioCrowd) it.next();
        VPoint first_pos = first_ped.getPosition();

        //iterate over markers in order to assign markers to a pedestrian and removes them from
        //markers, in order other pedestrians not to see them
        for (Iterator<Vector2D> it1 = markers.iterator(); it1.hasNext();){
            Vector2D marker = it1.next();
            double radius = 2.0;
            if (marker.distance(first_pos) < radius){
                first_ped.addMarker(marker);
                it1.remove();
            }
        }
    }
    public Vector2D avoid( Vector2D currentMov, PedestrianBioCrowd ped, List<Obstacle> obst){
        try {
            VPoint pos = ped.getPosition();
            Vector2D posVec = new Vector2D(ped.getPosition());
            VPoint posFut = new VPoint((posVec.add(currentMov)).x, (posVec.add(currentMov)).y);

            VShape closestIntersectingShape = null;
            double dist = 0;
            for (Obstacle obstacle : obst) {
                VLine pathLine = new VLine(pos, new VPoint((posVec.add(currentMov.multiply(10))).x, (posVec.add(currentMov.multiply(10))).y));

                if (obstacle.getShape().intersects(pathLine)) {
                    if (closestIntersectingShape == null) {
                        closestIntersectingShape = obstacle.getShape();
                        dist = obstacle.getShape().distance(pos);
                    } else if (dist > obstacle.getShape().distance(pos)) {
                        closestIntersectingShape = obstacle.getShape();
                        dist = obstacle.getShape().distance(pos);
                    }
                }

            }
            if (closestIntersectingShape == null) {
                return new Vector2D(0, 0);
            }

            if (closestIntersectingShape.distance(pos) < (attributesPedestrian.getMaximumSpeed())) {
                Vector2D mov = currentMov;
                if (pos.x > closestIntersectingShape.getCentroid().x) {
                    //ped is left of center
                    while (true) {
                        mov = mov.rotate(Math.PI * 2 - Math.PI / 20);
                        if (closestIntersectingShape.contains(new VPoint(mov.add(posVec).x, mov.add(posVec).y))) {
                            continue;
                        } else {
                            return mov.sub(currentMov);
                        }
                    }
                } else {
                    while (true) {
                        mov = mov.rotate(Math.PI / 6);
                        if (closestIntersectingShape.contains(new VPoint(mov.add(posVec).x, mov.add(posVec).y))) {
                        } else {
                            return mov.sub(currentMov);
                        }
                    }
                }
            }
        }catch (Exception e){
            System.out.println(e.getMessage());
        }
        return new Vector2D(0, 0);
    }
    public List<Obstacle> getObstacles() {
        //returns a List of all Obstacles
        Collection<Obstacle> obstacles = domain.getTopography().getObstacles();
        List<Obstacle> result = new LinkedList<>();
        for (Obstacle obstacle : obstacles) {
            result.add(obstacle);
        }
        return result;
    }
}
