package org.vadere.simulator.models.ocean.behaviour;

import org.vadere.simulator.models.ocean.PedestrianOcean;
import org.vadere.simulator.models.ocean.OceanSteeringModel;
import org.vadere.state.scenario.Obstacle;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.Iterator;
import java.util.List;

public class Decision {

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