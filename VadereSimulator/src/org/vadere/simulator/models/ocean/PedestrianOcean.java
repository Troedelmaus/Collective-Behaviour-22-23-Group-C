package org.vadere.simulator.models.ocean;

import org.vadere.state.attributes.scenario.AttributesAgent;
import org.vadere.state.scenario.Pedestrian;
import org.vadere.state.simulation.FootStep;
import org.vadere.util.geometry.shapes.VPoint;
import org.vadere.util.geometry.shapes.Vector2D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;
import java.io.*;


public class PedestrianOcean extends Pedestrian {

	private Vector2D lastMovement;
	private double[] oceanValues;
	private double[] outputs;
	private double[] memberO;
	private double[] memberC;
	private double[] memberE;
	private double[] memberEr;
	private double[] memberA;
	private double[] memberAr;
	private double[] memberN;

	private double startTime;
	private double lastSimTimeInSec;

	public PedestrianOcean(AttributesAgent attributesPedestrian, Random random) {
		super(attributesPedestrian, random);
		this.lastMovement = new Vector2D(0, 0);
		this.startTime = -1;
		this.lastSimTimeInSec = -1;

		this.oceanValues = new double[5];
		for (int i = 0; i < 5; i++) {
			oceanValues[i] = ((random.nextDouble() - 0.5) * 200); //Openness, Conscientiousness, Extroversion, Agreeableness and Neuroticism
		}
		this.memberO = initializeMembersip(oceanValues[0]);
		this.memberC = initializeMembersip(oceanValues[1]);
		this.memberE = initializeMembersip(oceanValues[2]);
		this.memberEr = initializeMembersip(oceanValues[2] * -1);
		this.memberA = initializeMembersip(oceanValues[3]);
		this.memberAr = initializeMembersip(oceanValues[3] * -1);
		this.memberN = initializeMembersip(oceanValues[4]);

		this.outputs = initializeOutputs();
		//System.out.println(Arrays.toString(oceanValues));
		//System.out.println(Arrays.toString(outputs));

	}

	public VPoint getLastMovement() {
		return lastMovement;
	}

	public double[] getOceanValues(){
		return outputs;
	}

	public double getStartTime() {
		return startTime;
	}

	public void move(double simTime, Vector2D mov) {
		lastMovement = mov;

		if (startTime < 0) {
			startTime = simTime;
			lastSimTimeInSec = 0;
		}

		VPoint oldPosition = getPosition();
		VPoint newPosition = oldPosition.add(mov);
		setPosition(newPosition);

		clearFootSteps();
		FootStep currentFootstep = new FootStep(oldPosition, newPosition, lastSimTimeInSec, simTime);
		getTrajectory().add(currentFootstep);
		getFootstepHistory().add(currentFootstep);

		lastSimTimeInSec = simTime;
	}

	public double[] initializeMembersip(double v) {
		return trapezoidMembership(v, -60, -20, 20, 60);
	}

	public double[] trapezoidMembership(double value, double a, double b, double c, double d ){
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

	public double trapezoidOutput(double[] o1, double[] o2, double x1, double x2, double x3, double x4, double x5, double x6){

		double ndlow = Math.max(Math.min(o1[0],o2[0]),Math.max(Math.min(o1[1],o2[0]),Math.min(o1[0],o2[1])));
		double ndmedium = Math.max(Math.min(o1[1],o2[1]),Math.max(Math.min(o1[2],o2[0]),Math.min(o1[0],o2[2])));
		double ndhigh = Math.max(Math.min(o1[1],o2[2]),Math.max(Math.min(o1[2],o2[1]),Math.min(o1[2],o2[2])));

		double[] areas = {(x2-x1)*ndlow, (x3-x2)*0.5*ndlow,
						  (x3-x2)*0.5 *ndmedium, (x4-x3)*ndmedium,
						  (x5-x4)*0.5*ndmedium,(x5-x4)*0.5*ndhigh,
						  (x6-x5)*ndhigh};
		double[] xCentroids = {(x2-x1)/2+x1,(x3-x2)/2+x2,(x3-x2)/2+x2,(x4-x3)/2+x3,(x5-x4)/2+x4,(x5-x4)/2+x4,(x6-x5)/2+x5};

		double outputTop = 0;
		double outputBottom = 0;

		for(int i = 0; i<7; i++){
			outputTop = outputTop + areas[i] * xCentroids[i];
			outputBottom = outputBottom + areas[i];
		}
		return outputTop/outputBottom;
	}

	public double[] initializeOutputs(){

		double nd = trapezoidOutput(memberO,memberC,3,6,12,18,24,30);//1.oceanV1,2.oceanV2,...list of x coords...
		double mn = trapezoidOutput(memberO,memberN,1,10,20,30,40,100);
		double th = trapezoidOutput(memberC,memberA,1,5,10,10,15,30);
		double r  = trapezoidOutput(memberEr,memberAr,0.3,0.5,1,1,1.5,2);
		double pv = trapezoidOutput(memberE,memberN,1.2,1.2,1.45,1.45,1.7,2.2);
		double [] outputs = {nd,mn,th,r,pv};
		return outputs;
	}
}
