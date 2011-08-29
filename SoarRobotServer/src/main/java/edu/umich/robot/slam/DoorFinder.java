package edu.umich.robot.slam;

import java.util.ArrayList;
import java.util.HashMap;

import april.config.Config;
import april.graph.GXYEdge;
import april.graph.GXYNode;
import april.graph.GXYTEdge;
import april.graph.GXYTNode;
import april.graph.Graph;
import april.jmat.LinAlg;
import april.jmat.MathUtil;

public class DoorFinder {

	/**
	 * The following parameters are used by the 'findDoors' method and may
	 * adjusted based on the environment.
	 * 
	 * @param maxPoints
	 *            When a possible beginning to a door has been found, the number
	 *            of lidar points do we search ahead for the end of the door.
	 * @param angThresh
	 *            Minimum angle threshold between two lidar points before
	 *            considering the points a possible edge of a door (see
	 *            locateDoors method for understanding of this angle
	 *            calculation).
	 * @param lowerAngThresh
	 *            Minimum double gated angle threshold between two lidar points
	 *            before considering a possible door (lidar scan sees a door but
	 *            nothing beyond the door, the two points considered by the
	 *            method below must be below this param before they will be
	 *            considered as a possible door).
	 * @param distThresh
	 *            Minimum distance between two lidar points before considering a
	 *            possible edge of a door.
	 * @param gapThresh
	 *            Minimum distance between two lidar points before considering a
	 *            possible door (lidar scan sees a door but nothing beyond the
	 *            door, the two points considered by the method below must be
	 *            below this parameter before they will be considered as a
	 *            possible door).
	 * @param doorDist
	 *            Minimum distance the considered door must be away before
	 *            actually classifying it as a door.
	 * @param windowThresh
	 *            Setting a window size in meters around the center of a
	 *            possible door. Used to check if other lidar points are within
	 *            this window.
	 * @param lowerDoorSize
	 *            Minimum size of a possible door before classifying it as a
	 *            door.
	 * @param upperDoorSize
	 *            Maximum size of a possible door before classifying it as a
	 *            door.
	 * @param linefitThresh
	 *            Maximum threshold of the linear regression used on the points
	 *            which make up a possible door.
	 * @param linePoints
	 *            Amount of additional points to use in the linear regression
	 *            algorithm. Add this number in front and behind the edges of
	 *            the door.
	 * @param maxDoors
	 *            Maximum amount of doors to find per scan.
	 */
	int maxPoints = 30;
	double angThresh = Math.toRadians(50);
	double distThresh = 0.05;
	double gapThresh = 2;
	double doorDist = 15;
	double windowThresh = 1;
	double lowerDoorSize = 1.0;
	double upperDoorSize = 5.0;
	double linefitThresh = 0.05;
	int linePoints = 4;
	int maxDoors = 3;

	/**
	 * The following parameters are used by the 'dataAssociation' method and may
	 * be adjusted based on environment or desired results.
	 * 
	 * @param upperDataThresh
	 *            Double gated upper threshold on the best data association
	 *            match. If outside this value, assign new landmark.
	 * @param lowerDataThresh
	 *            Double gated lower threshold on the best data association
	 *            match. If below this value, associate with corresponding
	 *            landmark.
	 */
	double upperDataThresh = 3;
	double lowerDataThresh = 0.5;

	// config file for door finder parameters
	Config config;

	// maps each door to it's index within graph.nodes
	HashMap<Integer, Integer> doorMap = new HashMap<Integer, Integer>();

	// number of doors that have been found
	int numDoors = 0;

	// used by the SLAM method to visualize door edges added to the graph
	int edgesAdded = 0;

	/**
	 * Constructor method (can be called with null as input).
	 * 
	 * @param config
	 *            Configuration file (see april.config for more information).
	 */
	public DoorFinder(Config config) {

		if (config != null) {
			this.maxPoints = config.getInt("maxPoints", maxPoints);
			this.angThresh = Math.toRadians(config.getDouble("angThresh",
					Math.toDegrees(angThresh)));
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
			this.upperDataThresh = config.getDouble("upperDataThresh",
					upperDataThresh);
			this.lowerDataThresh = config.getDouble("lowerDataThresh",
					lowerDataThresh);
		}

	}

	/**
	 * Main method for finding doors and adding them to the graph.
	 * 
	 * Within this method doors are located, associated, and updated within the
	 * SLAM pose graph.
	 * 
	 * @param scan
	 *            Set of lidar points in robot coordinate frame. Each entry
	 *            within the array list is a two element array of type double
	 *            representing a single lidar point. The first element is the
	 *            x-coordinate and the second element is the y-coordinate of the
	 *            lidar point.
	 * @param G
	 *            Graph being used for SLAM.
	 * @param PoseIndex
	 *            The index of the current pose within graph.nodes corresponding
	 *            to the lidar scan we are searching for doors within.
	 * @param curPose
	 *            The current pose of the robot (x, y, theta).
	 * @param matcher
	 *            The instance of the loop closing matcher used for SLAM.
	 */
	public void findDoors(ArrayList<double[]> scan, Graph G, int PoseIndex,
			double[] curPose, LoopClosureMatcher matcher) {

		// reset 'edgesAdded' for SLAM
		edgesAdded = 0;

		// find doors in the given scan
		ArrayList<double[]> doorsFound = locateDoors(scan, curPose);

		// do data associations
		int[] doorIndices = dataAssociation(doorsFound, G, PoseIndex, matcher);

		// update graph according to data association
		updateGraph(doorIndices, G, PoseIndex, curPose, doorsFound);

	}

	/**
	 * Finds doors within a given lidar scan.
	 * 
	 * Algorithm walks though every lidar point and attempts to find two edges
	 * of a door. Multiple thresholding occurs according to the parameters
	 * within this class (see above for thresholding parameters and their
	 * descriptions).
	 * 
	 * @param rpoints
	 *            Set of lidar points in robot coordinate frame. Each entry
	 *            within the array list is a two element array of type double
	 *            representing a single lidar point. The first element is the
	 *            x-coordinate and the second element is the y-coordinate of the
	 *            lidar point.
	 * @param xyt
	 *            Current location of the robot (x, y, theta).
	 * @return Returns an array list where each element within the list is a
	 *         three element array of type double representing a single door
	 *         found within the scan. The first element is the x-coordinate, the
	 *         second element is the y-coordinate, and the third element is the
	 *         orientation of the door in the global coordinate frame.
	 */
	public ArrayList<double[]> locateDoors(ArrayList<double[]> rpoints,
			double[] xyt) {

		// boolean flags
		boolean gap = true;
		boolean closeOp = false;

		// holds points for line fitting
		ArrayList<double[]> points = new ArrayList<double[]>();

		// holds located door locations (return variable)
		ArrayList<double[]> doors = new ArrayList<double[]>();

		// initializations prior to looping over lidar points
		int i = linePoints + 1;
		double[] curPoint = rpoints.get(i - 1);

		// searching through all lidar points for possible door edges
		while (i < rpoints.size()) {

			// next and last point in our scan
			double[] nextPoint = rpoints.get(i);
			double[] lastPoint = rpoints.get(i - 2);

			// distance between next point and our current point
			double dist = LinAlg.distance(nextPoint, curPoint, 2);

			// create vectors and determine the angle between them
			// v1 is the vector from lastPoint to curPoint
			// v2 is the vector from curPoint to nextPoint
			double[] v1 = LinAlg.subtract(curPoint, lastPoint);
			double[] v2 = LinAlg.subtract(nextPoint, curPoint);
			double ang = -MathUtil.mod2pi(Math.atan2(v2[1], v2[0])
					- Math.atan2(v1[1], v1[0]));

			// determining if we have found a possible edge for a door
			if ((ang > angThresh && dist > distThresh) || (dist > gapThresh)) {
				closeOp = true;
			} else {
				curPoint = LinAlg.copy(nextPoint);
				i = i + 1;
				continue;
			}

			// we have found an edge to a possible door, now looking for
			// the other edge to the door
			if (closeOp) {

				// searching ahead for the other edge of the door
				for (int j = 1; j < maxPoints; j++) {

					// avoid getting ahead of total number of points in scan
					if (j + i + linePoints + 1 > rpoints.size()) {
						break;
					}

					// current point we are considering as other door edge
					double[] closePoint = rpoints.get(j + i);

					// find other edge (same as first edge)
					double[] aheadPoint = rpoints.get(j + i + 1);
					double[] behindPoint = rpoints.get(j + i - 1);

					// distance between close point and ahead point
					dist = LinAlg.distance(behindPoint, closePoint, 2);

					// create vectors and determine the angle between them
					// v1 is the vector from behindPoint to
					// closePoint
					// v2 is the vector from closePoint to aheadPoint
					v1 = LinAlg.subtract(closePoint, behindPoint);
					v2 = LinAlg.subtract(aheadPoint, closePoint);
					ang = -MathUtil.mod2pi(Math.atan2(v2[1], v2[0])
							- Math.atan2(v1[1], v1[0]));

					// determining if we have found a possible edge for a door
					if ((ang > angThresh && dist > distThresh)
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
							points.add(rpoints.get(i - z - 1));

							// point ahead
							points.add(rpoints.get(i + j + z));
						}

						// fit this line
						double[] line = LinAlg.fitLine(points);

						// if line is close to being 'vertical', invert x/y and
						// recompute the linear regression
						if (Math.abs(line[0]) > 20) {

							ArrayList<double[]> newPoints = new ArrayList<double[]>();
							for (double[] pc : points) {
								double[] npc = new double[] { pc[1], pc[0] };
								newPoints.add(npc);
							}

							line = LinAlg.fitLine(newPoints);
							line[0] = (1 / line[0]);
						}

						// thresholding on the line fitting
						if (line[2] < linefitThresh) {

							// check for a true gap
							double xcenter = (curPoint[0] + closePoint[0]) / 2;
							double ycenter = (curPoint[1] + closePoint[1]) / 2;

							// looking for points between the two edges within
							// the window threshold of our gap
							for (int k = i + 2; k < i + j - 1; k++) {
								double[] gapPoint = rpoints.get(k);
								double xrange = Math.abs(gapPoint[0] - xcenter);
								double yrange = Math.abs(gapPoint[1] - ycenter);

								// final threshold check on the considered point
								// between the possible door edges
								if (xrange < windowThresh
										&& yrange < windowThresh) {

									// reset gap flag and break this loop to
									// continue searching for the other edge of
									// the door
									gap = false;
									break;
								}
							}

							// if there is a gap, continue verifying an actual
							// door has been found
							if (gap) {

								// lets test the distance of this gap now
								double gapDist = LinAlg.distance(closePoint,
										curPoint, 2);

								// checking size of door against
								if (gapDist > lowerDoorSize
										&& gapDist < upperDoorSize) {

									// checking how far away the door is
									if (Math.sqrt(Math.pow(xcenter, 2)
											+ Math.pow(ycenter, 2)) < doorDist) {

										// checking if we can see the door based
										// on our lidar scan (see method below)
										int numPoints = seeDoor(rpoints, i, i
												+ j, curPoint, closePoint);

										// checking how many points obstruct our
										// view
										if (numPoints == 0) {
											// finally found a door, determine
											// orientation of door
											double AofDoor = Math.atan2(
													-(1 / line[0]), 1);
											double[] dHolder = LinAlg
													.xytMultiply(xyt,
															new double[] {
																	xcenter,
																	ycenter,
																	AofDoor });
											dHolder[2] = MathUtil
													.mod2pi(dHolder[2]);

											// add this door to array list for
											// return
											doors.add(dHolder);

											// update index i for search
											i = i + j + 2;

											// reset flags and current point
											// considered
											closeOp = false;
											curPoint = rpoints.get(i - 1);
											break;
										}
									}
								}

							} else {
								// reset gap flag and continue search for other
								// edge of door
								gap = true;
							}

						}
					}
				}
			}

			// did we find a door
			if (!closeOp) {
				// how many doors were we allowed to find?
				if (doors.size() == maxDoors)
					return doors;
			}
			// we did not find a door, reset index and keep searching
			else {
				i = i + 1;
				curPoint = LinAlg.copy(nextPoint);
				closeOp = false;
			}
		}
		return doors;
	}

	/**
	 * Determines if the robot can see a particular door given a lidar scan.
	 * 
	 * To determine if any points are obstructing the view of the door, all
	 * points between the open and close edge of the door are tested against the
	 * closest point to the door.
	 * 
	 * @param scan
	 *            Set of lidar points in robot coordinate frame. Each entry
	 *            within the array list is a two element array of type double
	 *            representing a single lidar point. The first element is the
	 *            x-coordinate and the second element is the y-coordinate of the
	 *            lidar point.
	 * @param oPidx
	 *            Index within the lidar scan array list of the point considered
	 *            to be the open edge of the door.
	 * @param cPidx
	 *            Index within the lidar scan array list of the point considered
	 *            to be the close edge of the door.
	 * @param openPoint
	 *            Location of the point considered to be the open edge of the
	 *            door.
	 * @param closePoint
	 *            Location of the point considered to be the close edge of the
	 *            door.
	 * @return Returns the amount of points within the search cone closer to the
	 *         robot than the closest point of the door.
	 */
	public int seeDoor(ArrayList<double[]> scan, int oPidx, int cPidx,
			double[] openPoint, double[] closePoint) {

		// where to start search / end search
		int start = Math.min(oPidx, cPidx);
		int stop = Math.max(oPidx, cPidx);

		// determining the minimum value of the search cone
		double rangeOpen = Math.sqrt(Math.pow(openPoint[0], 2)
				+ Math.pow(openPoint[1], 2));
		double rangeClose = Math.sqrt(Math.pow(closePoint[0], 2)
				+ Math.pow(closePoint[1], 2));
		double minRange = Math.min(rangeOpen, rangeClose);

		// counter for the points within the search cone
		int pointsWithin = 0;

		// running through all points within the search cone to determine which
		// ones are closer to us than the door
		for (int i = start + 1; i < stop; i++) {
			double[] perpPoint = scan.get(i);
			double perpRange = Math.sqrt(Math.pow(perpPoint[0], 2)
					+ Math.pow(perpPoint[1], 2));

			if (perpRange < minRange)
				pointsWithin = pointsWithin + 1;
		}

		return pointsWithin;
	}

	/**
	 * Data association algorithm.
	 * 
	 * Determines from the newly located doors which doors correspond to
	 * previously seen doors.
	 * 
	 * @param daDoors
	 *            Array list of doors recently found in a lidar scan.
	 * @param graph
	 *            Graph being used for SLAM.
	 * @param poseidx
	 *            Index within graph.nodes of the current robot pose.
	 * @param matcher
	 *            The instance of the loop closing matcher used for SLAM.
	 * @return Returns an array list of type integer the same size as the number
	 *         of doors passed into the method via the door array list. Each
	 *         index within the returned array corresponds to the door of the
	 *         same index within the door array list. The integer values
	 *         returned represent the index within graph.nodes with which a
	 *         given door has been associated with. A value of '-1' corresponds
	 *         to a new door while a value of '-2' corresponds to an observation
	 *         of a door which should be ignored according to our double gated
	 *         parameters (see class parameters above).
	 */
	public int[] dataAssociation(ArrayList<double[]> daDoors, Graph graph,
			int poseidx, LoopClosureMatcher matcher) {

		// indices within graph of data associations
		int[] indices = new int[daDoors.size()];

		// no doors yet, so all doors are new and should be initialized
		if (doorMap.isEmpty()) {
			for (int i = 0; i < daDoors.size(); i++)
				indices[i] = -1;
			return indices;
		}

		// current pose in our map that has just been added
		// we are doing association on this pose's scan
		GXYTNode thisPose = (GXYTNode) graph.nodes.get(poseidx);

		// run through each door we have found to attempt association
		for (int j = 0; j < daDoors.size(); j++) {

			// current door we are trying to associate
			double[] doorLocation = daDoors.get(j);
			double best;
			int bestDoorindex;

			// initiate algorithm by associating the door we are considering
			// with the first door we ever saw
			int index = doorMap.get(0);

			// get the door node from the graph based on our door map (mapping
			// door number to index within graph.nodes)
			GXYNode doorNode = (GXYNode) graph.nodes.get(index);

			// grab the array list of all the door nodes that have seen the door
			// we are considering
			ArrayList<Integer> doorNodes = (ArrayList<Integer>) doorNode
					.getAttribute("nodeList");

			// get the index within graph.nodes of the last pose that saw the
			// door we are considering
			int lastNodeIdx = doorNodes.get(doorNodes.size() - 1);

			// current association heuristic: do a scan match between our
			// current pose and the pose that last saw the door we are trying to
			// associate with. our data association will be based on how well
			// the difference between the old edge connecting both poses and the
			// new edge from scan matching agrees with the error in the
			// association between the considered door and the located door
			GXYTNode lastPose = (GXYTNode) graph.nodes.get(lastNodeIdx);
			double[] prior = LinAlg.xytInvMul31(lastPose.state, thisPose.state);
			prior[2] = MathUtil.mod2pi(prior[2]);
			GXYTEdge ge = matcher.match(lastPose, thisPose, prior);
			double[] doorChange = new double[] {
					doorLocation[0] - doorNode.state[0],
					doorLocation[1] - doorNode.state[1] };
			if (ge != null) {
				double[] newChange = new double[] { prior[0] - ge.z[0],
						prior[1] - ge.z[1] };
				best = LinAlg.distance(newChange, doorChange, 2);
				bestDoorindex = index;
			} else {
				best = LinAlg.distance(doorLocation, doorNode.state, 2);
				bestDoorindex = index;
			}

			// run through the rest of the doors
			for (int k = 1; k < doorMap.size(); k++) {

				// run through each other door, determine if each associations
				// heuristic value is better than the current running 'best'
				index = doorMap.get(k);
				doorNode = (GXYNode) graph.nodes.get(index);
				doorNodes = (ArrayList<Integer>) doorNode
						.getAttribute("nodeList");
				lastNodeIdx = doorNodes.get(doorNodes.size() - 1);
				lastPose = (GXYTNode) graph.nodes.get(lastNodeIdx);
				prior = LinAlg.xytInvMul31(lastPose.state, thisPose.state);
				prior[2] = MathUtil.mod2pi(prior[2]);
				ge = matcher.match(lastPose, thisPose, prior);
				doorChange = new double[] {
						doorLocation[0] - doorNode.state[0],
						doorLocation[1] - doorNode.state[1] };
				double possBest;
				if (ge != null) {
					double[] newChange = new double[] { prior[0] - ge.z[0],
							prior[1] - ge.z[1] };
					possBest = LinAlg.distance(newChange, doorChange, 2);
				} else {
					possBest = LinAlg.distance(doorLocation, doorNode.state, 2);
				}
				if (possBest < best) {
					best = possBest;
					bestDoorindex = index;
				}
			}

			// double gate on our scan matching distance heuristic
			// new door to be added to the graph
			if (best > upperDataThresh) {
				indices[j] = -1;
				continue;
			}

			// associated door
			if (best < lowerDataThresh) {
				indices[j] = bestDoorindex;
				// update where we last saw the node
				ArrayList<Integer> nodeList = (ArrayList<Integer>) graph.nodes
						.get(bestDoorindex).getAttribute("nodeList");
				nodeList.add(poseidx);
				continue;
			}

			// ignore this observation (inside our gated thresholds)
			indices[j] = -2;
		}

		return indices;
	}

	/**
	 * Updates SLAM pose graph to include located doors.
	 * 
	 * Currently not set up to add additional edges based on successful data
	 * associations.
	 * 
	 * @param idxs
	 *            The indices from the data association method which associates
	 *            each door within the doors array list with a prior door. Each
	 *            value within the array corresponds either to a new door, the
	 *            index within graph.nodes of the associated door, or a '-2' to
	 *            designate an observation of a door which should be ignored
	 *            (all based on a double gates system, see above).
	 * @param gr
	 *            Graph being used for SLAM.
	 * @param poseidx
	 *            Index within graph.nodes of the current robot pose.
	 * @param xyt
	 *            Current location of the robot (x, y, theta).
	 * @param doors
	 *            Array list of doors recently found in a lidar scan.
	 */
	public void updateGraph(int[] idxs, Graph gr, int poseidx, double[] xyt,
			ArrayList<double[]> doors) {

		// run through each door, process it accordingly
		for (int i = 0; i < doors.size(); i++) {

			// door we are working with
			double[] doorState = doors.get(i);

			// initialize new door
			if (idxs[i] == -1) {

				// create the node, add to graph, update door map
				GXYNode doorNode = new GXYNode();
				doorNode.state = LinAlg.copy(doorState);
				doorNode.init = LinAlg.copy(doorState);
				doorNode.setAttribute("doorDirection", doorState[2]);
				ArrayList<Integer> nodeList = new ArrayList<Integer>();
				nodeList.add(poseidx);
				doorNode.setAttribute("nodeList", nodeList);
				doorMap.put(numDoors, gr.nodes.size());
				gr.nodes.add(doorNode);
				numDoors += 1;

				// create the edge between this door and the pose that saw this
				// door
				GXYEdge doorEdge = new GXYEdge();
				double dx = Math.cos(xyt[2]) * (doorNode.state[0] - xyt[0])
						+ Math.sin(xyt[2]) * (doorNode.state[1] - xyt[1]);
				double dy = -Math.sin(xyt[2]) * (doorNode.state[0] - xyt[0])
						+ Math.cos(xyt[2]) * (doorNode.state[1] - xyt[1]);
				doorEdge.z = new double[] { dx, dy };
				doorEdge.nodes = new int[] { poseidx, gr.nodes.size() - 1 };
				double xyDist = LinAlg.distance(doorNode.state, xyt, 2);
				doorEdge.P = LinAlg.diag(new double[] {
						LinAlg.sq(xyDist * 0.1) + 0.01,
						LinAlg.sq(xyDist * 0.1) + 0.01, });
				doorEdge.setAttribute("type", "door");
				gr.edges.add(doorEdge);
				edgesAdded += 1;
				continue;
			}
		}
	}
}