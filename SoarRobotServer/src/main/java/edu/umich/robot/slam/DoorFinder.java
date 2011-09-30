package edu.umich.robot.slam;

import java.util.ArrayList;

import april.config.Config;
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
	 *            of lidar points that are followed after the opening in hopes
	 *            to find the other edge to the door.
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
	 * @param lineFitThresh
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
	double lineFitThresh = 0.05;
	int linePoints = 4;
	int maxDoors = 3;

	/**
	 * The following parameters are used by the 'dataAssociation' method and may
	 * be adjusted based on environment or desired results.
	 * 
	 * @param upperAssociationThresh
	 *            Double gated upper threshold on the best data association
	 *            match. If outside this value, assign new landmark.
	 * @param lowerAssociationThresh
	 *            Double gated lower threshold on the best data association
	 *            match. If below this value, associate with corresponding
	 *            landmark.
	 */
	double upperAssociationThresh = 3;
	double lowerAssociationThresh = 0.5;

	// config file for door finder parameters
	Config config;

	// used by the SLAM method to visualize door edges added to the graph
	int edgesAdded = 0;

	// TODO Document Everything Below (until reaching constructors)

	ArrayList<Door> doors = new ArrayList<Door>();

	int lastDoorInside;
	double[] priorPose;
	boolean passedDoor = false;
	boolean inDoor = false;

	public static class Door {

		int doorNumber;

		int graphNodeIndex;

		int[] connectsRooms;

		ArrayList<Integer> poseNodes = new ArrayList<Integer>();
	}

	/**
	 * Constructor method (can be called with null as input).
	 * 
	 * @param config
	 *            Configuration file (see april.config for more information).
	 */
	public DoorFinder(Config config) {

		// configuration file exists
		if (config != null) {

			// updating door finding parameters
			this.maxPoints = config.getInt("maxPoints", maxPoints);
			this.angThresh = Math.toRadians(config.getDouble("angThresh",
					Math.toDegrees(angThresh)));
			this.distThresh = config.getDouble("distThresh", distThresh);
			this.gapThresh = config.getDouble("gapThresh", gapThresh);
			this.doorDist = config.getDouble("doorDist", doorDist);
			this.windowThresh = config.getDouble("windowThresh", windowThresh);
			this.lowerDoorSize = config.getDouble("lowerDoorSize",
					lowerDoorSize);
			this.upperDoorSize = config.getDouble("upperDoorSize",
					upperDoorSize);
			this.lineFitThresh = config.getDouble("lineFitThresh",
					lineFitThresh);
			this.linePoints = config.getInt("linePoints", linePoints);
			this.maxDoors = config.getInt("maxDoors", maxDoors);

			// updating data association parameters
			this.upperAssociationThresh = config.getDouble(
					"upperAssociationThresh", upperAssociationThresh);
			this.lowerAssociationThresh = config.getDouble(
					"lowerAssociationThresh", lowerAssociationThresh);
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
	 *            to the lidar scan that doors may reside within.
	 * @param curPose
	 *            The current pose of the robot (x, y, theta).
	 * @param matcher
	 *            The instance of the loop closing matcher used for SLAM.
	 */
	public void findDoors(ArrayList<double[]> scan, Graph G, int PoseIndex,
			double[] curPose, LoopClosureMatcher matcher, double poseThresh) {

		// reset 'edgesAdded' for SLAM GUI
		edgesAdded = 0;

		// find doors in the given scan
		ArrayList<double[]> doorsFound = locateDoors(scan, curPose);

		// do data associations
		int[] doorIndices = dataAssociation(doorsFound, G, PoseIndex, matcher);

		// update graph according to data association
		updateGraph(doorIndices, G, PoseIndex, curPose, doorsFound);

		passedThroughDoor(G, curPose, poseThresh);
		
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

			// determining if a possible edge for a door has been found
			if ((ang > angThresh && dist > distThresh) || (dist > gapThresh)) {
				closeOp = true;
			} else {
				curPoint = LinAlg.copy(nextPoint);
				i = i + 1;
				continue;
			}

			// an edge to a possible door has been found, now looking for
			// the other edge to the door
			if (closeOp) {

				// searching ahead for the other edge of the door
				for (int j = 1; j < maxPoints; j++) {

					// avoid getting ahead of total number of points in scan
					if (j + i + linePoints + 1 > rpoints.size()) {
						break;
					}

					// current point being considered as the other door edge
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

					// determining if there is an edge to a door
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
						if (line[2] < lineFitThresh) {

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

										// checking if the robot can see the
										// door based on the LIDAR scan (see
										// method below)
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

											double[] newDoor = new double[] {
													dHolder[0], dHolder[1],
													dHolder[2], gapDist };

											// add this door to array list for
											// return
											doors.add(newDoor);

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

			// has a door been found
			if (!closeOp) {

				// how many doors in this scan can be found
				if (doors.size() == maxDoors)
					return doors;
			}
			// a door was not found, reset index and keep searching
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
	 *            Array list of doors recently found in a LIDAR scan.
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

	// TODO only associate with doors within our current room
	public int[] dataAssociation(ArrayList<double[]> daDoors, Graph graph,
			int poseidx, LoopClosureMatcher matcher) {

		// indices within graph of data associations
		int[] indices = new int[daDoors.size()];

		// no doors yet, so all doors are new and should be initialized
		if (doors.isEmpty()) {
			for (int i = 0; i < daDoors.size(); i++)
				indices[i] = -1;
			return indices;
		}

		// current pose in our map that has just been added
		GXYTNode currentPose = (GXYTNode) graph.nodes.get(poseidx);

		// run through each door found and attempt association with prior doors
		for (int j = 0; j < daDoors.size(); j++) {

			// current door attempting to associate
			double[] doorLocation = daDoors.get(j);
			double bestVal;
			int bestDoor;

			// initiate algorithm by assuming association with the first door
			// ever found
			Door curDoor = doors.get(0);

			// get the door node from the graph based on our door map (mapping
			// door number to index within graph.nodes)
			GXYTNode doorNode = (GXYTNode) graph.nodes
					.get(curDoor.graphNodeIndex);

			// get the index within graph.nodes of the last pose that saw the
			// door being considered for association
			int lastNodeIdx = curDoor.poseNodes
					.get(curDoor.poseNodes.size() - 1);

			GXYTNode lastPose = (GXYTNode) graph.nodes.get(lastNodeIdx);

			bestVal = associationHueristic(currentPose, lastPose, doorLocation,
					doorNode.state, matcher);

			bestDoor = curDoor.doorNumber;

			// run through the rest of the doors
			for (int p = 1; p < doors.size(); p++) {

				curDoor = doors.get(p);

				// run through each other door, determine if each associations
				// heuristic value is better than the current running 'best'
				doorNode = (GXYTNode) graph.nodes.get(curDoor.graphNodeIndex);

				lastNodeIdx = curDoor.poseNodes
						.get(curDoor.poseNodes.size() - 1);

				lastPose = (GXYTNode) graph.nodes.get(lastNodeIdx);

				double possBest = associationHueristic(currentPose, lastPose,
						doorLocation, doorNode.state, matcher);

				if (possBest < bestVal) {
					bestVal = possBest;
					bestDoor = curDoor.doorNumber;
				}
			}

			// double gate on our scan matching distance heuristic
			// new door to be added to the graph
			if (bestVal > upperAssociationThresh) {
				indices[j] = -1;
				continue;
			}

			// associated door
			if (bestVal < lowerAssociationThresh) {
				indices[j] = bestDoor;
				continue;
			}

			// ignore this observation (inside gated thresholds)
			indices[j] = -2;
		}

		return indices;
	}

	// TODO document this method
	public double associationHueristic(GXYTNode curPose, GXYTNode oldPose,
			double[] curDoorLocation, double[] oldDoorLocation,
			LoopClosureMatcher matcher) {

		double[] prior = LinAlg.xytInvMul31(curPose.state, oldPose.state);
		prior[2] = MathUtil.mod2pi(prior[2]);
		GXYTEdge ge = matcher.match(curPose, oldPose, prior);
		double[] assocDiff = new double[] {
				oldDoorLocation[0] - curDoorLocation[0],
				oldDoorLocation[1] - curDoorLocation[1] };

		if (ge != null) {
			double[] updateEdge = LinAlg.xytInverse(ge.z);
			double[] newPoseLocation = LinAlg.xytMultiply(oldPose.state,
					updateEdge);
			double[] poseLocatDiff = new double[] {
					newPoseLocation[0] - curPose.state[0],
					newPoseLocation[1] - curPose.state[1] };

			return LinAlg.distance(poseLocatDiff, assocDiff);
		}

		return Math.sqrt(Math.pow(assocDiff[0], 2) + Math.pow(assocDiff[1], 2));
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
			ArrayList<double[]> foundDoors) {

		// run through each door, process it accordingly
		for (int i = 0; i < foundDoors.size(); i++) {

			// current considered door
			double[] doorState = foundDoors.get(i);

			// door associated, update node list of the given door
			if (idxs[i] > -1) {
				Door assocDoor = doors.get(idxs[i]);
				assocDoor.poseNodes.add(poseidx);
			}

			// initialize new door
			if (idxs[i] == -1) {

				// create the door object
				Door newDoor = new Door();
				newDoor.doorNumber = doors.size();
				newDoor.graphNodeIndex = gr.nodes.size();
				newDoor.poseNodes.add(poseidx);
				doors.add(newDoor);

				// create the node, add to graph, update door map
				GXYTNode doorNode = new GXYTNode();
				doorNode.state = LinAlg.copy(doorState, 3);
				doorNode.init = LinAlg.copy(doorState, 3);
				doorNode.setAttribute("type", "door");
				doorNode.setAttribute("doorWidth", doorState[3]);
				gr.nodes.add(doorNode);

				// create the edge between this door and the pose that saw this
				// door
				GXYTEdge doorEdge = new GXYTEdge();
				doorEdge.z = LinAlg.xytInvMul31(xyt, doorNode.state);
				doorEdge.nodes = new int[] { poseidx, gr.nodes.size() - 1 };
				doorEdge.P = LinAlg.diag(new double[] { 0.01, 0.01, 0.001 });
				doorEdge.setAttribute("type", "door");
				gr.edges.add(doorEdge);
				edgesAdded += 1;
				continue;
			}
		}
	}

	// TODO DOCUMENT
	public boolean insideDoor(Door thisDoor, Graph gr, double[] pose,
			double poseThresh) {

		// get door, door's directionality, door's width
		GXYTNode doorNode = (GXYTNode) gr.nodes.get(thisDoor.graphNodeIndex);
		double doorDir = doorNode.state[2];
		double doorNormDir = MathUtil.mod2pi(doorDir + (Math.PI / 2));
		double doorWidth = (Double) doorNode.getAttribute("doorWidth");

		// calculate rectangular window points
		double dirXval1 = doorNode.state[0] + poseThresh
				* Math.cos(doorDir);
		double dirXval2 = doorNode.state[0] - poseThresh
				* Math.cos(doorDir);
		double norXval1 = doorNode.state[0] + (doorWidth / 2)
				* Math.cos(doorNormDir);
		double norXval2 = doorNode.state[0] - (doorWidth / 2)
				* Math.cos(doorNormDir);
		double[] xvals = new double[] { dirXval1, dirXval2, norXval1, norXval2 };

		// calculate y values of rectangular points
		double dirYval1 = doorNode.state[1] + poseThresh
				* Math.sin(doorDir);
		double dirYval2 = doorNode.state[1] - poseThresh
				* Math.sin(doorDir);
		double norYval1 = doorNode.state[1] + (doorWidth / 2)
				* Math.sin(doorNormDir);
		double norYval2 = doorNode.state[1] - (doorWidth / 2)
				* Math.sin(doorNormDir);
		double[] yvals = new double[] { dirYval1, dirYval2, norYval1, norYval2 };

		// determine if pose within the window
		if (pose[0] <= LinAlg.max(xvals) && pose[0] >= LinAlg.min(xvals)) {
			if (pose[1] <= LinAlg.max(yvals) && pose[1] >= LinAlg.min(yvals)) {
				return true;
			}
		}

		return false;
	}

	// TODO Document && eventually only doors connecting the room we are in
	public void passedThroughDoor(Graph gr, double[] curPose, double poseThresh) {
		
		// if last pose not in door, lets see if this pose is
		if (!inDoor) {
			for (Door doorCheck : doors) {
				boolean inDoorNow = insideDoor(doorCheck, gr, curPose,
						poseThresh);
				if (inDoorNow) {
					lastDoorInside = doorCheck.doorNumber;
					inDoor = true;
					priorPose = LinAlg.copy(curPose);
					return;
				}
			}
			return;
		}

		// last pose was in a door, check if we are still in that door
		if (!insideDoor(doors.get(lastDoorInside), gr, curPose, poseThresh)) {
			
			// reset inside door
			inDoor = false;

			// checking to see if now on the other side of the door we were in
			// grab door
			GXYTNode doorCross = (GXYTNode) gr.nodes.get(doors
					.get(lastDoorInside).graphNodeIndex);

			// last pose and current pose locations WRT door's reference frame
			double[] beforeInDoor = LinAlg.xytInvMul31(doorCross.state, priorPose);
			double[] afterInDoor = LinAlg.xytInvMul31(doorCross.state, curPose);
			
			if((afterInDoor[0] > 0 && beforeInDoor[0] < 0) || (afterInDoor[0] < 0 && beforeInDoor[0] > 0)){
				System.out.println("WE CROSSED A DOOR!!!!!!!!!!!!!");
			}
			
		}

	}
}