package org.vadere.gui.components.control.simulation;

import org.vadere.gui.components.model.DefaultSimulationConfig;
import org.vadere.gui.components.model.SimulationModel;

import javax.swing.*;
import java.awt.*;

public class ActionSetAerosolCloudColor extends ActionSetColor {

    public ActionSetAerosolCloudColor(String name, SimulationModel<? extends DefaultSimulationConfig> model, JPanel coloredPanel){
        super(name, model, coloredPanel);
    }

    @Override
    protected void saveColor(Color color) { model.config.setAerosolCloudColor(color); }
}