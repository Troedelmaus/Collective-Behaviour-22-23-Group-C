package org.vadere.simulator.projects.dataprocessing.processor;

import org.apache.commons.lang3.tuple.Pair;
import org.vadere.annotation.factories.dataprocessors.DataProcessorClass;
import org.vadere.simulator.control.simulation.SimulationState;
import org.vadere.simulator.projects.dataprocessing.ProcessorManager;
import org.vadere.simulator.projects.dataprocessing.datakey.PedestrianIdKey;
import org.vadere.simulator.projects.dataprocessing.datakey.TimeRectangleGridKey;
import org.vadere.simulator.projects.dataprocessing.procesordata.PedestrianIdDuration;
import org.vadere.state.attributes.processor.AttributesAreaDensityGridSpatialTimeProcessor;
import org.vadere.state.attributes.processor.AttributesProcessor;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.simulation.FootStep;
import org.vadere.state.simulation.VTrajectory;
import org.vadere.util.geometry.LinkedCellsGrid;
import org.vadere.util.geometry.shapes.VLine;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.VRectangle;
import org.vadere.util.logging.Logger;

import java.util.*;

@DataProcessorClass(label = "AreaDensityGridSpatialTimeProcessor")
public class AreaDensityGridSpatialTimeProcessor extends DataProcessor<TimeRectangleGridKey, Double> {

    private double nextTime;

    private double timeWindowSize;

    private LinkedCellsGrid<PedestrianIdDuration> cellsElements;

    private AttributesAreaDensityGridSpatialTimeProcessor attr;

    private PedestrianTrajectoryProcessor trajectoryProcessor;

    private static Logger logger = Logger.getLogger(AreaDensityGridSpatialTimeProcessor.class);

    public AreaDensityGridSpatialTimeProcessor() {
        super("gridDensity");
        setAttributes(new AttributesAreaDensityGridSpatialTimeProcessor());
    }

    @Override
    protected void doUpdate(SimulationState state) {
        this.trajectoryProcessor.update(state);
        if (state.getSimTimeInSec() >= nextTime + timeWindowSize) {
            cellsElements.clear();
            Collection<Pedestrian> peds = state.getTopography().getPedestrianDynamicElements().getElements();
            for (Pedestrian ped : peds) {
                LinkedList<FootStep> stepsInWindow = getFootstepsForTimeWindow(ped);
                computeTraversedCellsAndDurations(stepsInWindow, ped);
            }

            Map<int[], List<PedestrianIdDuration>> elementsByCell = cellsElements.getElementsByCell();
            for (Map.Entry<int[], List<PedestrianIdDuration>> result: elementsByCell.entrySet()) {
                double totalPedTimeInCell = result.getValue().stream()
                        .map(r -> r.getDuration())
                        .mapToDouble(d -> d)
                        .sum();
                VRectangle gridCell = cellsElements.getGridCellAsRectangle(result.getKey()[0], result.getKey()[1]);
                double cellDensity = totalPedTimeInCell / (gridCell.getArea() * timeWindowSize); // see duives 2015
                double time = nextTime + (timeWindowSize / 2);
                if (totalPedTimeInCell > 0.0) {
                    logger.debug(time + ": total Time in cell " + Arrays.toString(result.getKey()) + ": " + totalPedTimeInCell);
                }
                TimeRectangleGridKey key = new TimeRectangleGridKey(time,  gridCell.x, gridCell.y, gridCell.width, gridCell.height);
                this.putValue(key, cellDensity);
            }
            nextTime += timeWindowSize;
        }
    }

    private void computeTraversedCellsAndDurations(LinkedList<FootStep> stepsInWindow, Pedestrian ped) {
        if (!stepsInWindow.isEmpty()) {
            // set start time of footstep to start of timewindow
            FootStep fs = stepsInWindow.get(0);
            FootStep fsPrev= stepsInWindow.get(0);
            double startTime = Math.max(fs.getStartTime(), nextTime);
            VPoint startPoint;
            if (startTime != fs.getStartTime()) {
                startPoint = footStepApproximatePointForTime(startTime, fs);
                fs = new FootStep(startPoint, fs.getEnd(), startTime, fs.getEndTime());
                stepsInWindow.set(0, fs);
            }

            int[] gridIdx = cellsElements.gridPos(fs.getStart());
            VRectangle currentCell = cellsElements.getGridCellAsRectangle(gridIdx[0], gridIdx[1]);
            double timeInCell = 0.0;
            Pair<VRectangle, Double> fsResult;
            for (int i = 0; i < stepsInWindow.size(); i++) {
                if (fs.getStart() != fsPrev.getEnd() || fs.getStartTime() != fsPrev.getEndTime()) {
                    logger.debug("next footstep did start at different position and/or time. fs: "
                            + i + "/" + (stepsInWindow.size() - 1));
                    FootStep intermediate = new FootStep(fsPrev.getEnd(), fs.getStart(), fsPrev.getEndTime(), fs.getStartTime());
                    fsResult = computeCellForFootStep(intermediate, currentCell, timeInCell, gridIdx, ped);
                    currentCell = fsResult.getLeft();
                    timeInCell = fsResult.getRight();
                }
                fsResult = computeCellForFootStep(fs, currentCell, timeInCell, gridIdx, ped);
                currentCell = fsResult.getLeft();
                timeInCell = fsResult.getRight();
                // next footstep of ped
                if (i + 1 < stepsInWindow.size()) {
                    fsPrev = fs;
                    fs = stepsInWindow.get(i + 1);
                }
            }
        }
    }

    private Pair<VRectangle, Double> computeCellForFootStep(FootStep fs, VRectangle cell, double timeInCell,
                                                 int[] gridIdx, Pedestrian ped) {
        if (cell.contains(fs.getEnd())) {
            // still same cell
            timeInCell += Math.min(fs.getEndTime(), nextTime + timeWindowSize) - fs.getStartTime();
            timeInCell = timeInCell;
        } else {

            double exitingTime = fs.computeIntersectionTime(cell);
            //add rest time in cell
            timeInCell += Math.min(exitingTime, nextTime + timeWindowSize) - fs.getStartTime();
            //add pedestrian with total duration for cell
            cellsElements.addObject(new PedestrianIdDuration(ped.getId(), timeInCell, cell));

            if (exitingTime < nextTime + timeWindowSize) {
                // check for skipped cells during one footstep
                List<VRectangle> skipped = computeSkippedCells(gridIdx, cellsElements.gridPos(fs.getEnd()), cell);
                gridIdx = cellsElements.gridPos(fs.getEnd());
                cell = cellsElements.getGridCellAsRectangle(gridIdx[0], gridIdx[1]); //next cell for ped
                if (!cell.contains(fs.getEnd())) {
                    logger.warn("computed cell " + cell + " for point " + fs.getEnd() + " did not contain it. ");
                }
                double newCellEnteringTime = fs.computeIntersectionTime(cell);

                if (!skipped.isEmpty()) {
                    double skippedDuration = Math.min(newCellEnteringTime, nextTime + timeWindowSize)
                            - exitingTime;
                    for (VRectangle intermediateCell : skipped) {
                        //add every cell in between with a shared amount of time
                        cellsElements.addObject(new PedestrianIdDuration(ped.getId(),
                                skippedDuration / skipped.size(), intermediateCell));
                        logger.debug(ped + " skipped grid cells during one footstep: " + skippedDuration + " s");
                    }
                }

                //new cell duration of footstep entering
                if (newCellEnteringTime < nextTime + timeWindowSize) {
                    timeInCell = Math.min(fs.getEndTime(), nextTime + timeWindowSize) - newCellEnteringTime;
                } else {
                    timeInCell = 0.0;
                }
            }
        }
        return Pair.of(cell, timeInCell);
    }

    private VPoint footStepApproximatePointForTime(double time, FootStep footStep) {
        assert time < footStep.getEndTime();
        double ratio = (time - footStep.getStartTime()) / footStep.duration();

        VLine line = new VLine(footStep.getStart(), footStep.getEnd());
        return line.midPoint(-0.5 + ratio);
    }

    private LinkedList<FootStep> getFootstepsForTimeWindow(Pedestrian pedestrian) {
        VTrajectory pedTrajectory = this.trajectoryProcessor.getValue(new PedestrianIdKey(pedestrian.getId()));
        Iterator<FootStep> it = pedTrajectory.getFootSteps()
                .descendingIterator();

        LinkedList<FootStep> stepsInWindow = new LinkedList<>();
        while (it.hasNext()) {
            FootStep footStep = it.next();
            stepsInWindow.addFirst(footStep);
            if (footStep.getStartTime() <= nextTime) {
                if (footStep.getEndTime() <= nextTime) {
                    // latest step before time Window
                    stepsInWindow.removeFirst();
                }
                break;
            }
            if (nextTime + timeWindowSize <= footStep.getEndTime()) {
                if (stepsInWindow.size() > 1) {
                    stepsInWindow.removeLast();
                }
            }
        }
        return stepsInWindow;
    }

    private List<VRectangle> computeSkippedCells(int[] currentIdx, int[] newIdx, VRectangle currentCell) {
        int iXDiff = currentIdx[0] - newIdx[0];
        int iYDiff = currentIdx[1] - newIdx[1];
        List<VRectangle> skippedCellsCenters = new ArrayList<>();
        double cellWidthX = currentCell.getWidth() * Integer.signum(iXDiff);
        double cellWidthY = currentCell.getHeight() * Integer.signum(iYDiff);
        for (int i = 1; i < Math.abs(iXDiff); i++) {
            for (int j = 1; j < Math.abs(iYDiff); j++) {
                double intermediateX = currentCell.x + i * cellWidthX;
                double intermediateY = currentCell.y + j * cellWidthY;
                skippedCellsCenters.add(new VRectangle(intermediateX, intermediateY, currentCell.width, currentCell.height));
            }
        }
        return skippedCellsCenters;
    }

    @Override
    public void preLoop(SimulationState state) {
        super.preLoop(state);
        cellsElements = new LinkedCellsGrid<>(
                state.getTopography().getBounds().x,
                state.getTopography().getBounds().y,
                state.getTopography().getBounds().width,
                state.getTopography().getBounds().height,
                attr.getCellSize());
    }


    @Override
    public void init(ProcessorManager manager) {
        super.init(manager);
        this.nextTime = 0.0;
        attr = (AttributesAreaDensityGridSpatialTimeProcessor) getAttributes();
        this.timeWindowSize = attr.getT();
        this.trajectoryProcessor = (PedestrianTrajectoryProcessor) manager.getProcessor(attr.getPedestrianTrajectoryProcessorId());
    }

    @Override
    public AttributesProcessor getAttributes() {
        if (super.getAttributes() == null) {
            setAttributes(new AttributesAreaDensityGridSpatialTimeProcessor());
        }
        return super.getAttributes();
    }
}
