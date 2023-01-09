package org.vadere.simulator.models.biocrowds;

import org.vadere.state.attributes.Attributes;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.ArrayList;

public class AttributesBioCrowd extends Attributes {

    public VPoint target;

    public ArrayList<Vector2D> getMarkers() {
        return markers;
    }

    private final ArrayList<Vector2D> markers;

    public AttributesBioCrowd() {
        this.markers = new ArrayList<Vector2D>();
        this.target = new VPoint(14.5, 23.1);
        ScatterMarkers();
    }

    private void ScatterMarkers() {

        int x_min = 7;
        int x_max = 22;

        int y_min = 7;
        int y_max = 22;

        for (int i = 0; i < 100; i++){
            double random_x = Math.random() * (x_max - x_min + 1) + x_min;
            double random_y = Math.random() * (y_max - y_min + 1) + y_min;
            this.markers.add(new Vector2D(random_x, random_y));
        }
//        this.markers.add(new Vector2D(8.0, 8.0));
//        this.markers.add(new Vector2D(15.0, 15.0));
//        this.markers.add(new Vector2D(this.target.x, this.target.y));
    }

}
