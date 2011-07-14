package edu.umich.robot.slam;

import java.util.ArrayList;
import java.util.HashMap;

import april.config.Config;
import april.graph.GXYEdge;
import april.graph.GXYNode;
import april.graph.Graph;
import april.jmat.LinAlg;
import april.jmat.MathUtil;

public class DoorFinder {
	// config file for door finder parameters
	Config config;

	// door finder parameters
	int maxPoints = 30;
	double upperAngThresh = Math.toRadians(60);
	double lowerAngThresh = Math.toRadians(5);
	double distThresh = 0.05;
	double gapThresh = 2;
	double doorDist = 15;
	double windowThresh = 1;
	double lowerDoorSize = 3.0;
	double upperDoorSize = 5.0;
	double linefitThresh = 0.05;
	int linePoints = 4;
	int maxDoors = 2;
	double distErr = 0.15;
	
	// booleans for door detection (and some other const. variables)
	boolean gap = true;
	boolean closeOp = false;
	ArrayList<double[]> points = new ArrayList<double[]>();
	ArrayList<double[]> doors = new ArrayList<double[]>();
	
	//door finder slam parameters
	boolean doorSlam = true;
	HashMap<Integer, Integer> doorMap = new HashMap<Integer, Integer>();
	int numDoors = 0;
	int edgesAdded = 0;

	// constructors
	public DoorFinder() {
		this(null);
	}

	public DoorFinder(Config config) {

		if (config != null) {
			this.maxPoints = config.getInt("maxPoints", maxPoints);
			this.upperAngThresh = Math.toRadians(config.getDouble(
					"upperAngThresh", Math.toDegrees(upperAngThresh)));
			this.lowerAngThresh = Math.toRadians(config.getDouble(
					"lowerAngThresh", Math.toDegrees(lowerAngThresh)));
			this.distThresh = config.getDouble("distThresh", distThresh);
			this.gapThresh = config.getDouble("gapThresh", gapThresh);
			this.doorDist = config.getDouble("doorDist", doorDist);
			this.windowThresh = config.getDouble("windowThresh", windowThresh);
			this.linePoints = config.getInt("linePoints", linePoints);
			this.maxDoors = config.getInt("maxDoors", maxDoors);
			this.lowerDoorSize = config.getDouble("lowerDoorSize",
					lowerDoorSize);
			this.upperDoorSize = config.getDouble("upperDoorSize",
					upperDoorSize);
			this.linefitThresh = config.getDouble("linefitThresh",
					linefitThresh);
			this.doorSlam = config.getBoolean("doorSlam", doorSlam);
			this.distErr = config.getDouble("distErr", distErr);
		}

	}
	
	// main method for finding doors and adding them to our graph
	public void findDoors(ArrayList<double[]> scan, Graph G, int PoseIndex, double[] curPose){
		
		//clear door array list and reset edgesAdded
		doors.clear(); 
		edgesAdded = 0;
		
		//find doors in the given scan
		locateDoors(scan, curPose);
		
		//do data associations
		int[] doorIndices = dataAssociation(doors, G);
		
		//update graph according to data association
		updateGraph(doorIndices, G, PoseIndex, curPose);
		
	}

	// find doors
	public void locateDoors(ArrayList<double[]> rpoints, double[] xyt) {

		// initializations prior to looping over rpoints
		int i = linePoints + 1;
		double[] curPoint = rpoints.get(i - 1);

		// searching through all points
		while (i < rpoints.size()) {

			// next and last point in our scan
			double[] nextPoint = rpoints.get(i);
			double[] lastPoint = rpoints.get(i - 2);

			// distance between this next point and our current point
			double dist = LinAlg.distance(nextPoint, curPoint);

			// create vectors and determine the angel between them
			// v1 is the vector from lastPoint to curPoint
			// v2 is the vector from curPoint to nextPoint
			double[] v1 = LinAlg.subtract(curPoint, lastPoint);
			double[] v2 = LinAlg.subtract(nextPoint, curPoint);
			double ang = -MathUtil.mod2pi(Math.atan2(v2[1], v2[0])
					- Math.atan2(v1[1], v1[0]));

			// have we found a 'drop-off'?
			if ((ang > upperAngThresh && dist > distThresh)
					|| (dist > gapThresh)) {
				closeOp = true;
			} else {
				curPoint = LinAlg.copy(nextPoint);
				i = i + 1;
				continue;
			}

			if (closeOp) {

				// we are now attempting to find the close point of a potential
				// door
				for (int j = 1; j < maxPoints; j++) {

					// make sure we don't get ahead of our points
					if (j + i + linePoints + 1 > rpoints.size()) {
						break;
					}

					// current point we are attempting to close door with
					double[] closePoint = rpoints.get(j + i);

					// find next possible close point (same as finding open
					// point)
					double[] aheadPoint = rpoints.get(j + i + 1);
					double[] behindPoint = rpoints.get(j + i - 1);

					// distance between close point and ahead point
					dist = LinAlg.distance(behindPoint, closePoint);

					// create vectors and determine the angle between them
					// v1 is the vector from behindPoint to
					// closePoint
					// v2 is the vector from closePoint to aheadPoint
					v1 = LinAlg.subtract(closePoint, behindPoint);
					v2 = LinAlg.subtract(aheadPoint, closePoint);
					ang = -MathUtil.mod2pi(Math.atan2(v2[1], v2[0])
							- Math.atan2(v1[1], v1[0]));

					// check thresholds, is this a possible ending point for our
					// door?
					if ((ang > upperAngThresh && dist > distThresh)
							|| (dist > gapThresh)) {

						// clean out array list points for line fitting
						points.clear();

						// points to attempt to fit a line to
						points.add(lastPoint);
						points.add(curPoint);
						points.add(closePoint);
						points.add(aheadPoint);

						// additional points for line fitting (padding our line)
						for (int z = 2; z < linePoints + 1; z++) {
							// point behind
							points.add(rpoints.get(i - z));

							// point ahead
							points.add(rpoints.get(i + j + z));
						}

						// fit this line
						double[] line = LinAlg.fitLine(points);

						// is the line too close to vertical to get a good
						// estimation from?
						if (Math.abs(line[0]) > 20) {

							// lets switch the axis recompute the regression
							ArrayList<double[]> newPoints = new ArrayList<double[]>();
							for (double[] pc : points) {
								double[] npc = new double[] { pc[1], pc[0] };
								newPoints.add(npc);
							}

							// new line now we have switched axis
							line = LinAlg.fitLine(newPoints);
						}

						// thresholding on the line fitting
						if (line[2] < linefitThresh) {

							// check there really is a gap here
							double xcenter = (curPoint[0] + closePoint[0]) / 2;
							double ycenter = (curPoint[1] + closePoint[1]) / 2;

							// any of these other points in our gap?
							for (int k = i + 2; k < i + j - 1; k++) {
								double[] gapPoint = rpoints.get(k);
								double xrange = Math.abs(gapPoint[0] - xcenter);
								double yrange = Math.abs(gapPoint[1] - ycenter);

								// found a point in our gap! (not a door)
								if (xrange < windowThresh
										&& yrange < windowThresh) {
									gap = false;
									break;
								}
							}

							// if there is a gap, continue checks for door
							if (gap) {

								// lets test the distance of this gap now
								double gapDist = LinAlg.distance(closePoint,
										curPoint);

								// are we within our door size?
								if (gapDist > lowerDoorSize
										&& gapDist < upperDoorSize) {

									// lets make sure the door is somewhat close
									// to
									// us
									if (Math.sqrt(Math.pow(xcenter, 2)
											+ Math.pow(ycenter, 2)) < doorDist) {

										// can we really see this door?
										int numPoints = seePoint(rpoints, i, i
												+ j, curPoint, closePoint);
										if (numPoints == 0) {
											// we have finally found a door,
											// break this loop, reset our point
											// search index 'i', set closeOp
											// back to 0
											double[] dHolder = LinAlg.xytMultiply(xyt, new double[] {
													xcenter, ycenter, 0 });
											doors.add(dHolder);
											i = i + j + 2;
											closeOp = false;
											curPoint = rpoints.get(i - 1);
											break;
										}
									}
								}

							} else {
								gap = true;
							}

						}
					}
				}
			}

			// did we find a door?
			if (!closeOp) {
				// how many doors were we allowed to find?
				if (doors.size() == maxDoors)
					return;
			}
			// we did not find a door...keep searching
			else {
				i = i + 1;
				curPoint = LinAlg.copy(nextPoint);
				closeOp = false;
			}
		}
	}

	//used by find doors
	public int seePoint(ArrayList<double[]> scan, int oPidx, int cPidx,
			double[] openPoint, double[] closePoint) {

		// where to start search / end search
		int start = Math.min(oPidx, cPidx);
		int stop = Math.max(oPidx, cPidx);

		// what is min range of the search cone?
		double rangeOpen = Math.sqrt(Math.pow(openPoint[0], 2)
				+ Math.pow(openPoint[1], 2));
		double rangeClose = Math.sqrt(Math.pow(closePoint[0], 2)
				+ Math.pow(closePoint[1], 2));
		double minRange = Math.min(rangeOpen, rangeClose);

		// how many points are within this cone?
		int pointsWithin = 0;

		// lets find some perpetrators!
		for (int i = start + 1; i < stop; i++) {
			double[] perpPoint = scan.get(i);
			double perpRange = Math.sqrt(Math.pow(perpPoint[0], 2)
					+ Math.pow(perpPoint[1], 2));

			if (perpRange < minRange)
				pointsWithin = pointsWithin + 1;
		}

		return pointsWithin;
	}
	
	//do data association on found doors
	public int[] dataAssociation(ArrayList<double[]> daDoors, Graph graph) {
		
		//indices within graph of data associations (-1 if new door)
		int[] indices = new int[daDoors.size()];
		
		//no doors yet, so all doors are new and should be initialized
		if(doorMap.isEmpty()){
			for(int i = 0; i < daDoors.size(); i++)
				indices[i] = -1;
			return indices;
		}
		
		//doing very simple nearest neighbor data association
		for(int j = 0; j < daDoors.size(); j++){

			//current door we are trying to do assocation on
			double[] doorLocation = daDoors.get(j);
			
			// initializations
			int index = doorMap.get(0);
			GXYNode oldDoor = (GXYNode) graph.nodes.get(index);
			double dist = LinAlg.distance(oldDoor.state, doorLocation);
			double bestDist = dist;
			int bestDoorindex = index;

			// simple nearest neighbor (not using probability for now...)
			for (int k = 1; k < doorMap.size(); k++) {

				// grab door index and door node information
				index = doorMap.get(k);
				oldDoor = (GXYNode) graph.nodes.get(index);

				// calculate distance from the door found to considered door
				dist = LinAlg.distance(oldDoor.state, doorLocation);

				// best found so far?
				if (dist < bestDist) {
					bestDist = dist;
					bestDoorindex = index;
				}
			}

			if (bestDist > 5) {
				indices[j] = -1;
			}
			
			else{
				indices[j] = bestDoorindex;
			}
		}
		
		return indices;
	}
	
	//update the graph to reflect doors (as well as slam edges if applicable)
	public void updateGraph(int[] idxs, Graph gr, int poseidx, double[] xyt){
		
		//run through each door, process it accordingly
		for(int i = 0; i < doors.size(); i++){
			
			//door we are working with
			double[] doorState = doors.get(i);
			
			//new door, initialize regardless slam setup
			if(idxs[i] == -1){
				
				//create the node, add to graph, update door map
				GXYNode doorNode = new GXYNode();
				doorNode.state = LinAlg.copy(doorState);
				doorNode.init = LinAlg.copy(doorState);
				doorMap.put(numDoors, gr.nodes.size());
				gr.nodes.add(doorNode);
				numDoors += 1;
				
				//create the edge between this door and the pose that saw this door
				GXYEdge doorEdge = new GXYEdge();
				double dx = Math.cos(xyt[2])* (doorNode.state[0] - xyt[0])+ Math.sin(xyt[2])* (doorNode.state[1] - xyt[1]);
				double dy = -Math.sin(xyt[2])* (doorNode.state[0] - xyt[0])+ Math.cos(xyt[2])* (doorNode.state[1] - xyt[1]);
				doorEdge.z = new double[] {dx, dy};
				doorEdge.nodes = new int[] {poseidx, gr.nodes.size() - 1};
				double xyDist = LinAlg.distance(doorNode.state, xyt, 2);
				doorEdge.P = LinAlg.diag(new double[] {
						LinAlg.sq(xyDist * distErr) + 0.01,
						LinAlg.sq(xyDist * distErr) + 0.01, });
				doorEdge.setAttribute("type", "door");
				gr.edges.add(doorEdge);
				edgesAdded += 1;
				continue;
			}
			
			//not a new door, add edge if using doors to assist in slam
			if(doorSlam){
				GXYEdge doorEdge = new GXYEdge();
				double dx = Math.cos(xyt[2]) * (doorState[0] - xyt[0]) + Math.sin(xyt[2]) * (doorState[1] - xyt[1]);
				double dy = -Math.sin(xyt[2]) * (doorState[0] - xyt[0]) + Math.cos(xyt[2]) * (doorState[1] - xyt[1]);
				doorEdge.z = new double[] { dx, dy };
				double xyDist = LinAlg.distance(doorState, xyt, 2);
				doorEdge.P = LinAlg.diag(new double[] {
						LinAlg.sq(xyDist * distErr) + 0.01,
						LinAlg.sq(xyDist * distErr) + 0.01, });
				doorEdge.nodes = new int[] { poseidx, idxs[i]};
				doorEdge.setAttribute("type", "door");
				gr.edges.add(doorEdge);
				edgesAdded += 1;
			}
		}
	}

	//change slam settings on door finder class
	public void setSlamMethod(boolean option){
		this.doorSlam = option;
	}
}
