package edu.umich.robot.slam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Random;

import april.config.Config;
import april.graph.CholeskySolver;
import april.graph.DijkstraProjection;
import april.graph.GEdge;
import april.graph.GNode;
import april.graph.GXYNode;
import april.graph.GXYTEdge;
import april.graph.GXYTNode;
import april.graph.GXYTPosEdge;
import april.graph.Graph;
import april.image.SigProc;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.MultiGaussian;
import april.jmat.ordering.MinimumDegreeOrdering;
import april.laser.scanmatcher.MultiResolutionScanMatcher;
import april.util.GridMap;

public class Slam {

	/**
	 * The following booleans control the overall SLAM setup.The configuration
	 * files hold the variables for each process. The various objects
	 * instantiate other classes used for SLAM.
	 * 
	 * @param useOdom
	 *            Set to true if using odometry sensors.
	 * @param closeLoop
	 *            Set to true to attempt to close loops.
	 * @param noisyOdom
	 *            Set to true to artificially inflate the odometry information
	 *            with error (used for simulation environment).
	 * @param includeDoors
	 *            Set to true to attempt to locate doors and add them to the
	 *            pose graph.
	 */
	boolean useOdom = true;
	boolean closeLoop = true;
	boolean noisyOdom = true;
	boolean includeDoors = true;
	Config slamConfig;
	Config loopMatcherConfig;
	Config loopCloserConfig;
	Config doorFinderConfig;
	DoorFinder doorFinder;
	LoopClosureMatcher loopmatcher;
	MultiResolutionScanMatcher openMatch;
	Graph g = new Graph();

	/**
	 * The following variables/parameters control various aspects of the SLAM
	 * process. See each description for further information.
	 * 
	 * @variable lastPoseIndex
	 * @variable openGridMap
	 * @variable distErr
	 * @variable rotErr
	 * @variable poseDistThresh
	 * @variable poseRotThresh
	 * @variable randomGen
	 * @variable samplingError
	 * @variable trueOdom
	 * @variable pureOdom
	 * @variable slamOdom
	 * @variable starter
	 * @variable decimate
	 * @variable decimateCounter
	 * @variable lastScan
	 * @variable xyt
	 * @variable groundTruth
	 * @variable XYT
	 * @variable addedEdges
	 * 
	 */
	int lastPoseIndex = 0;
	GridMap openGridMap;
	double distErr = 0.15;
	double rotErr = 0.1;
	double poseDistThresh = 1.0;
	double poseRotThresh = Math.toRadians(25);
	Random randomGen = new Random();
	double[] samplingError = new double[] { 0.005, 0.005, Math.toRadians(0.05) };
	ArrayList<double[]> trueOdom = new ArrayList<double[]>();
	ArrayList<double[]> pureOdom = new ArrayList<double[]>();
	ArrayList<double[]> slamOdom = new ArrayList<double[]>();
	int starter = 0;
	int decimate = 1;
	public int decimateCounter;
	ArrayList<double[]> lastScan;
	double xyt[] = new double[3];
	double groundTruth[] = new double[3];
	double XYT[] = new double[3];
	ArrayList<GEdge> addedEdges = new ArrayList<GEdge>();

	/**
	 * Open loop scan matcher parameters / variables used only if the SLAM
	 * routine does not use odometry sensors. Do not confuse these with the open
	 * loop matching routine within the loop closure matcher. Those parameters
	 * are to be set within the configuration file passed to the loop closure
	 * matcher in it's constructor call below.
	 * 
	 * @param openMetersPerPixel
	 *            Resolution on the grid map below (meters per pixel).
	 * @param openSearchRange
	 *            Search window size (in meters).
	 * @param openSearchTheta
	 *            Rotation search space (in radians).
	 * @param openSearchThetaRes
	 *            Rotation search space resolution (in radians).
	 * @param openGridMapSize
	 *            Grid map size (in meters).
	 * @param scanDecay
	 *            Amount of decay in the open loop grid map per pose added to
	 *            the graph.
	 */
	double openMetersPerPixel = 0.05;
	double openSearchRange = 0.2;
	double openSearchTheta = Math.toRadians(15);
	double openSearchThetaRes = Math.toRadians(1);
	double openGridMapSize = 20;
	int scanDecay = 5;

	/**
	 * Loop closure SLAM process parameters. Adjust based on desired
	 * performance, environment, and robot setup. These parameters are passed
	 * into the loop closure matcher whenever a close loop match is being
	 * processed. All parameters within the configuration file for the loop
	 * closure matcher should be set for the open loop matching process which
	 * attempts to gain a better estimate on an odometry edge (see slam routine
	 * below for further understanding).
	 * 
	 * @param closeMatchScore
	 *            Minimum score that must be achieved by the multi-resolution
	 *            scan matcher before an matching edge is returned.
	 * @param closeThetaRange
	 *            Rotation search space (in radians).
	 * @param closeThetaRangeRes
	 *            Rotation search space resolution (in radians).
	 * @param closeSearchRange
	 *            Search window size (in meters).
	 */
	double closeMatchScore = 8000;
	double closeThetaRange = Math.toRadians(10);
	double closeThetaRangeRes = Math.toRadians(1);
	double closeSearchRange = 10;

	/**
	 * The following parameters / variables are for the loop closing method
	 * 'closer' below.
	 * 
	 * @variable lastNode Index of the last node that was processed for loop
	 *           closures.
	 * @variable lastEdge Index of the last edge that was created from open loop
	 *           techniques.
	 * @param maxLoopLength
	 *            Maximum loop length of the depth first search.
	 * @param maxMatchAttempts
	 *            Maximum scan match attempts per loop closing.
	 * @param maxAddedEdges
	 *            Maximum number of edges to add per node in the graph.
	 * @param minTrace
	 *            TODO Edit this description.
	 * @param maxMahal
	 *            TODO Edit this description.
	 * @param hypoThresh
	 *            TODO Edit this description.
	 * @param closeFreq
	 *            Specifies how often to attempt to close the loop (Every Nth
	 *            number of nodes)
	 * @variable hypoNodes Hypotheses grouped according to the nodes they
	 *           connect. Includes odometry edges.
	 */
	int lastNode = 2;
	int lastEdge = 0;
	int maxLoopLength = 4;
	int maxMatchAttempts = 40;
	int maxAddedEdges = 10;
	double minTrace = 0.2;
	double maxMahal = 1.0;
	double hypoThresh = 5.0;
	double closeFreq = 1;
	HashMap<Integer, ArrayList<GXYTEdge>> hypoNodes = new HashMap<Integer, ArrayList<GXYTEdge>>();

	/**
	 * Laser scan class used by open loop scan matcher when no odometry
	 * information is availalbe.
	 * 
	 */
	public static class Scan {
		// pose of the robot at the time of scan
		double xyt[];

		// LIDAR scan points in global frame
		ArrayList<double[]> gpoints;
	}

	/**
	 * Constructor method (can be called with null as input).
	 * 
	 * @param config
	 *            Configuration file (see april.config for more information).
	 */
	public Slam(Config config) {

		// parsing configuration file
		if (config != null) {
			slamConfig = config.getChild("Slam");
			loopMatcherConfig = config.getChild("LoopMatcher");
			loopCloserConfig = config.getChild("LoopCloser");
			doorFinderConfig = config.getChild("DoorFinder");

			// setting SLAM configuration parameters
			decimate = slamConfig.getInt("decimate", decimate);
			poseDistThresh = slamConfig.getDouble("poseDistThresh",
					poseDistThresh);
			poseRotThresh = Math.toRadians(slamConfig.getDouble(
					"poseRotThresh", Math.toDegrees(poseRotThresh)));
			distErr = slamConfig.getDouble("distErr", distErr);
			rotErr = slamConfig.getDouble("rotErr", rotErr);
			useOdom = slamConfig.getBoolean("useOdom", useOdom);
			closeLoop = slamConfig.getBoolean("closeLoop", closeLoop);
			noisyOdom = slamConfig.getBoolean("noisyOdom", noisyOdom);
			includeDoors = slamConfig.getBoolean("includeDoors", includeDoors);
			closeMatchScore = slamConfig.getDouble("closeMatchScore",
					closeMatchScore);
			closeThetaRange = Math.toRadians(slamConfig.getDouble(
					"closeThetaRange", closeThetaRange));
			closeThetaRangeRes = Math.toRadians(slamConfig.getDouble(
					"closeThetaRangeRes", closeThetaRangeRes));
			closeSearchRange = slamConfig.getDouble("closeSearchRange",
					closeSearchRange);
			openMetersPerPixel = slamConfig.getDouble("openMetersPerPixel",
					openMetersPerPixel);
			openSearchRange = slamConfig.getDouble("openSearchRange",
					openSearchRange);
			openSearchTheta = Math.toRadians(slamConfig.getDouble(
					"openSearchTheta", openSearchTheta));
			openSearchThetaRes = Math.toRadians(slamConfig.getDouble(
					"openSearchThetaRes", openSearchThetaRes));
			openGridMapSize = slamConfig.getDouble("openGridMapSize",
					openGridMapSize);
			scanDecay = slamConfig.getInt("scanDecay", scanDecay);

			// setting loop closer configuration parameters
			maxLoopLength = loopCloserConfig.getInt("maxLoopLength",
					maxLoopLength);
			maxMatchAttempts = loopCloserConfig.getInt("maxMatchAttempts",
					maxMatchAttempts);
			maxAddedEdges = loopCloserConfig.getInt("maxAddedEdges",
					maxAddedEdges);
			minTrace = loopCloserConfig.getDouble("minTrace", minTrace);
			maxMahal = loopCloserConfig.getDouble("maxMahal", maxMahal);
			hypoThresh = loopCloserConfig.getDouble("hypoThresh", hypoThresh);
			closeFreq = loopCloserConfig.getDouble("closeFreq", closeFreq);
		}

		// initializations based on SLAM setup
		if (!useOdom) {
			openMatch = new MultiResolutionScanMatcher(new Config());
			openGridMap = GridMap.makeMeters(-(openGridMapSize / 2),
					-(openGridMapSize / 2), openGridMapSize, openGridMapSize,
					openMetersPerPixel, 0);
			starter = 1;
		}

		if (includeDoors) {
			doorFinder = new DoorFinder(doorFinderConfig);
		}

		g = new Graph();
	}

	/**
	 * Simple updating of the current pose estimate using odometry sensor
	 * information.
	 * 
	 * @param odomxyt
	 *            Pose estimate from odometry sensors, pass in as [x, y, theta]
	 *            in global coordinate frame (units are meters / radians).
	 */
	public void processOdometry(double odomxyt[]) {

		// not using odometry according to SLAM setup, return (see above)
		if (!useOdom)
			return;

		// map the current angle between -pi to pi
		odomxyt[2] = MathUtil.mod2pi(odomxyt[2]);

		// allow for an initial offset position
		if (starter == 0) {
			xyt = LinAlg.copy(odomxyt);
			XYT = LinAlg.copy(odomxyt);
			groundTruth = LinAlg.copy(odomxyt);
			synchronized (trueOdom) {
				trueOdom.add(LinAlg.copy(odomxyt));
			}
			synchronized (pureOdom) {
				pureOdom.add(LinAlg.copy(odomxyt));
			}
			synchronized (slamOdom) {
				slamOdom.add(LinAlg.copy(odomxyt));
			}
			starter++;
			return;
		}

		// sampling from noisy odometry according to SLAM setup (see above)
		if (noisyOdom) {
			// noisy sampling from odometry using rigid body transformation
			// process model
			double dist = LinAlg.distance(groundTruth, odomxyt);
			double dang = Math
					.abs(MathUtil.mod2pi(groundTruth[2] - odomxyt[2]));

			// only adding error in odometry measurement if we have moved since
			// the last time we received a message
			if (dist > 0.00001 || dang > 0.00001) {
				// synchronizing for GUI threading
				synchronized (trueOdom) {
					trueOdom.add(LinAlg.copy(odomxyt));
				}

				// true movement RBT parameters
				double[] priorU = LinAlg.xytInvMul31(groundTruth, odomxyt);
				priorU[2] = MathUtil.mod2pi(priorU[2]);
				groundTruth = LinAlg.copy(odomxyt);

				// sampling from noise parameters, assuming error of al[num] per
				// each full unit of true movement
				double s1 = Math.abs(samplingError[0] * priorU[0]);
				double s2 = Math.abs(samplingError[1] * priorU[1]);
				double s3 = Math.abs(samplingError[2] * priorU[2]);

				double Dx = priorU[0] + Math.sqrt(s1)
						* randomGen.nextGaussian();
				double Dy = priorU[1] + Math.sqrt(s2)
						* randomGen.nextGaussian();
				double Dt = priorU[2] + Math.sqrt(s3)
						* randomGen.nextGaussian();

				// new odomxyt after sampling from noisy motion model
				double[] newU = new double[] { Dx, Dy, Dt };
				odomxyt = LinAlg.xytMultiply(XYT, newU);
				odomxyt[2] = MathUtil.mod2pi(odomxyt[2]);

				synchronized (pureOdom) {
					pureOdom.add(LinAlg.copy(odomxyt));
				}
			} else {
				return;
			}
		}

		// rigid body transformation update (XYT stores the previous odometry
		// sensor measurement)
		double[] u = LinAlg.xytInvMul31(XYT, odomxyt);

		// Reassign XYT values
		XYT = LinAlg.copy(odomxyt);

		// simple updating of the predicted pose
		xyt = LinAlg.xytMultiply(xyt, u);
		xyt[2] = MathUtil.mod2pi(xyt[2]);
	}

	/** Pass in points relative to the robot's coordinate, units of 'meters' **/
	public void processScan(ArrayList<double[]> rpoints) {
		// used by GUI for displaying scans
		lastScan = rpoints;

		// wait for first position message to arrive before beginning scan
		// matching
		if (starter == 0)
			return;

		// first scan received after the initial odometry message has been
		// received
		if (g.nodes.isEmpty()) {

			// first scan ever, add a node to the graph
			GXYTNode gn = new GXYTNode();
			gn.state = xyt;
			gn.init = xyt;

			// anchor this node in the graph using a GXYTPosEdge (see
			// GXYTPosEdge for more information)
			GXYTPosEdge edge = new GXYTPosEdge();
			edge.nodes = new int[] { 0 };
			edge.z = LinAlg.copy(gn.state);
			edge.P = LinAlg.diag(new double[] { 0.01, 0.01, 0.001 });
			g.edges.add(edge);

			// add attributes (LIDAR scan and node index within graph.nodes)
			gn.setAttribute("points", rpoints);
			gn.setAttribute("node_index", g.nodes.size());

			// add node to graph
			g.nodes.add(gn);

			// update the open loop grid map if we are not using odometry sensor
			// information
			if (!useOdom) {
				Scan scan = new Scan();
				scan.xyt = xyt;
				scan.gpoints = LinAlg.transform(xyt, rpoints);
				drawScan(scan);
			}

			// call main door finder method to find doors, associate the data,
			// and add them to the graph
			if (includeDoors) {
				doorFinder.findDoors(rpoints, g, lastPoseIndex, xyt,
						loopmatcher);

				// add door edges to our addedEdges array list for visualization
				// (see SlamGui for more information)
				for (int d = 0; d < doorFinder.edgesAdded; d++)
					addedEdges.add(g.edges.get(g.edges.size() - 1 - d));
			}

			return;
		}

		// throwing away scan based upon parameters in declaration above
		decimateCounter++;
		if (decimateCounter != decimate)
			return;
		decimateCounter = 0;

		// scan matcher using prior scans if not using odometry
		if (!useOdom) {
			MultiGaussian posterior = openMatch.match(rpoints, xyt, null,
					openSearchRange, openSearchRange, openSearchTheta,
					openSearchThetaRes);
			double[] positionUpdate = posterior.getMean();
			xyt = LinAlg.copy(positionUpdate);
		}

		// where was our last pose
		GXYTNode NA = (GXYTNode) g.nodes.get(lastPoseIndex);
		double ddist = LinAlg.distance(xyt, NA.state, 2);
		double dtheta = Math.abs(MathUtil.mod2pi(xyt[2] - NA.state[2]));

		if (ddist > poseDistThresh || dtheta > poseRotThresh) {

			// create two edges, an odometry edge and scan match edge, use the
			// best of the two based on whether or not the match meets all scan
			// matching requirements
			GXYTNode gn = new GXYTNode();
			gn.state = xyt;
			gn.init = xyt;
			gn.setAttribute("points", rpoints);
			gn.setAttribute("node_index", g.nodes.size());
			g.nodes.add(gn);

			// new edge using just odometry information
			GXYTEdge ge = new GXYTEdge();
			ge.z = LinAlg.xytInvMul31(NA.state, xyt);
			ge.P = LinAlg.diag(new double[] {
					LinAlg.sq(ddist * distErr) + 0.01,
					LinAlg.sq(ddist * distErr) + 0.01,
					LinAlg.sq(dtheta * rotErr) + Math.toRadians(1) });
			ge.nodes = new int[2];
			ge.nodes[0] = lastPoseIndex;
			ge.nodes[1] = g.nodes.size() - 1;

			// new edge using scan matching information
			loopmatcher = new LoopClosureMatcher(loopMatcherConfig);

			// initialize the loop closure matcher with the current LIDAR scan
			loopmatcher.init(rpoints);

			// current pose
			GXYTNode NB = (GXYTNode) g.nodes.get(g.nodes.size() - 1);

			// creating a prior (just odometry edge)
			double[] prior = LinAlg.xytInvMul31(NB.state, NA.state);

			// setting the loop matcher to open loop to enforce a bound on the
			// matching (this avoids the hallway problem)
			loopmatcher.setOpenLoopMatch(true);

			// invoking the match (note that this is called in a reverse manner
			// in an effort to use the same instance of the loop closure matcher
			// during the loop closing process below)
			GXYTEdge newEdge = loopmatcher.match(NB, NA, prior);

			// setting the loop matcher back to close loop for loop closure
			// below
			loopmatcher.setOpenLoopMatch(false);

			// found a good match, fix current node and add matching edge to
			// graph.edges
			if (newEdge != null) {

				// invert this edge (see matching scheme above for info), update
				// all of the edge's information, add to graph.edges, update
				// current pose's node
				newEdge.z = LinAlg.xytInverse(newEdge.z);
				newEdge.nodes = new int[2];
				newEdge.nodes[0] = lastPoseIndex;
				newEdge.nodes[1] = g.nodes.size() - 1;
				double[] tempXYZ = LinAlg.xytMultiply(NA.state, newEdge.z);
				gn.init = tempXYZ;
				gn.state = tempXYZ;
				double newdist = LinAlg.distance(gn.state, NA.state, 2);
				double newtheta = Math.abs(MathUtil.mod2pi(gn.state[2]
						- NA.state[2]));
				newEdge.P = LinAlg.diag(new double[] {
						LinAlg.sq(newdist * distErr) + 0.01,
						LinAlg.sq(newdist * distErr) + 0.01,
						LinAlg.sq(newtheta * rotErr) + Math.toRadians(1) });
				g.edges.add(newEdge);
				xyt = LinAlg.copy(tempXYZ);
			} else {
				g.edges.add(ge);
			}

			// switch scan matcher search information for loop closing
			loopmatcher.setMatchParameters(closeMatchScore, closeThetaRange,
					closeThetaRangeRes, closeSearchRange);

			// attempt to find doors based on SLAM setup
			if (includeDoors) {
				lastPoseIndex = g.nodes.size() - 1;
				doorFinder.findDoors(rpoints, g, lastPoseIndex, xyt,
						loopmatcher);
				for (int d = 0; d < doorFinder.edgesAdded; d++)
					addedEdges.add(g.edges.get(g.edges.size() - 1 - d));
			}

			// attempt to close the loop based on SLAM setup
			if (closeLoop) {
				int curEdges = g.edges.size();
				close();
				if (curEdges < g.edges.size()) {
					optimizeFull(g);
					xyt = LinAlg.copy(g.nodes.get(lastPoseIndex).state);
				}
			}

			// updating the open loop grid map if we are not using odometry
			// information
			if (!useOdom) {
				Scan scan = new Scan();
				scan.xyt = xyt;
				scan.gpoints = LinAlg.transform(xyt, rpoints);
				drawScan(scan);
			}
		}

		// add updated pose to the list of SLAM poses
		synchronized (slamOdom) {
			slamOdom.add(LinAlg.copy(xyt));
		}
	}

	/**
	 * Updates the open loop scan matching grid map which is utilized only when
	 * odometry information is not available.
	 * 
	 * @param s
	 *            Current Scan object created from the most recent pose and
	 *            LIDAR scan (see above for the actual Scan class definition)
	 */
	void drawScan(Scan s) {
		double minx = Double.MAX_VALUE, maxx = -Double.MAX_VALUE;
		double miny = Double.MAX_VALUE, maxy = -Double.MAX_VALUE;

		for (double p[] : s.gpoints) {
			minx = Math.min(minx, p[0]);
			maxx = Math.max(maxx, p[0]);
			miny = Math.min(miny, p[1]);
			maxy = Math.max(maxy, p[1]);
		}

		openGridMap.recenter((minx + maxx) / 2, (miny + maxy) / 2, 5);

		openGridMap.subtract(scanDecay);

		ArrayList<double[]> gpoints = s.gpoints;
		double[] last_lp = new double[1];
		double lastpidx = 0;
		;
		for (int pidx = 0; pidx < gpoints.size(); pidx++) {
			double lp[] = gpoints.get(pidx);
			openGridMap.setValue(lp[0], lp[1], (byte) 255);

			if (last_lp != null && lastpidx == pidx - 1) {
				double dist = LinAlg.distance(lp, last_lp);
				if (dist < 0.25) {
					openGridMap.drawLine(last_lp[0], last_lp[1], lp[0], lp[1],
							(byte) 255);
				}
			}

			last_lp = lp;
			lastpidx = pidx;
		}

		float f[] = SigProc.makeGaussianFilter(1.5, 5);
		f = LinAlg.scale(f, 1.0 / LinAlg.max(f));
		openGridMap.filterFactoredCenteredMax(f, f);

		openMatch.setModel(openGridMap);
	}

	/**
	 * ADD HYPOTHESIS USED BY MAIN LOOP CLOSING METHOD
	 * 
	 * @param ge
	 */
	void addHypothesis(GXYTEdge ge) {
		ArrayList<GXYTEdge> a = hypoNodes.get(ge.nodes[0]);
		if (a == null) {
			a = new ArrayList<GXYTEdge>();
			hypoNodes.put(ge.nodes[0], a);
		}
		a.add(ge);

		ArrayList<GXYTEdge> b = hypoNodes.get(ge.nodes[1]);
		if (b == null) {
			b = new ArrayList<GXYTEdge>();
			hypoNodes.put(ge.nodes[1], b);
		}
		b.add(ge);
	}

	/**
	 * Main loop closing method.
	 */
	public void close() {

		// need to fix the autoclose last node shit
		lastNode = lastPoseIndex;

		// nothing to do
		if (includeDoors) {
			if (g == null || (g.nodes.size() - doorFinder.numDoors) <= 2) {
				return;
			}
			if (lastNode == g.nodes.size()) {
				return;
			}
		} else {
			if (g == null || g.nodes.size() <= 2) {
				return;
			}
			if (lastNode == g.nodes.size()) {
				return;
			}
		}

		// add other XYT edges to our hypothesis pool
		synchronized (g) {
			for (; lastEdge < g.edges.size(); lastEdge++) {
				GEdge _ge = g.edges.get(lastEdge);
				String type = (String) _ge.getAttribute("type");
				if (type == null)
					type = "odom";

				if (!type.equals("auto") && (_ge instanceof GXYTEdge)) {
					GXYTEdge ge = (GXYTEdge) _ge;
					addHypothesis(ge);
				}
			}
		}

		DijkstraProjection dp;

		// generate edges between node "autoclose_last" and earlier nodes.
		GXYTNode refnode = (GXYTNode) g.nodes.get(lastNode);

		ArrayList<GXYTEdge> candidates = new ArrayList<GXYTEdge>();

		synchronized (g) {
			dp = new DijkstraProjection(g,
					(Integer) refnode.getAttribute("node_index"));
		}

		// identify the set of nodes that we'd like to do scanmatching
		// to.
		for (int i = 0; i < lastNode; i++) {
			GXYTEdge dpPrior = dp.getEdge(i);

			// only want edges which attach poses to poses
			if (dpPrior == null) {
				continue;
			}

			if (g.nodes.get(dpPrior.nodes[0]) instanceof GXYNode
					|| g.nodes.get(dpPrior.nodes[1]) instanceof GXYNode) {
				continue;
			}

			// which nodes *might* be in the neighborhood?
			// Compute the mahalanobis distance that the nodes are
			// within 5 meters of each other.
			// HACK - add additional structure to requirements of
			// possible candidates:
			// forcing the candidates to be within a "true" distance of
			// the maximum
			// xy search area of the loop closure matcher parameter
			double curDist = LinAlg.distance(refnode.state,
					g.nodes.get(dpPrior.nodes[1]).state, 2);
			if (curDist > loopmatcher.getSearchDist())
				continue;

			// mahalanobis distance to add candidates for loop closure
			double mahl = mahalanobisDistance(dpPrior, hypoThresh);
			if (mahl < 0.45)
				candidates.add(dpPrior);
		}

		// shuffle, shuffle, shuffle...
		Collections.shuffle(candidates);

		for (int cidx = 0; cidx < Math.min(maxMatchAttempts, candidates.size()); cidx++) {
			GXYTEdge dpPrior = candidates.get(cidx);

			// attempting to match nodes
			GXYTNode nodeA = (GXYTNode) g.nodes.get(dpPrior.nodes[0]);
			GXYTNode nodeB = (GXYTNode) g.nodes.get(dpPrior.nodes[1]);
			double[] prior = LinAlg.xytInvMul31(nodeA.state, nodeB.state);
			prior[2] = MathUtil.mod2pi(prior[2]);
			GXYTEdge ge = loopmatcher.match(nodeA, nodeB, prior);

			if (ge != null) {
				ge.z[2] = MathUtil.mod2pi(ge.z[2]);
				ge.nodes = new int[] { dpPrior.nodes[0], dpPrior.nodes[1] };
				ge.setAttribute("type", "auto");
				addHypothesis(ge);
			}

		}

		// cumulative motion around loop
		double xytl[] = new double[3];

		// edges that we've already taken (so we don't use the same edge
		// twice)
		HashSet<GXYTEdge> visitedEdges = new HashSet<GXYTEdge>();

		// poses that we've already visited
		int pidxs[] = new int[maxLoopLength];
		pidxs[0] = lastNode;

		ArrayList<GXYTEdge> acceptedEdges = new ArrayList<GXYTEdge>();

		recurse(1, pidxs, visitedEdges, xytl, acceptedEdges);

		synchronized (g) {

			for (int i = 0; i < maxAddedEdges; i++) {
				GXYTEdge bestedge = null;
				double besttrace = minTrace;

				for (GXYTEdge ge : acceptedEdges) {
					HashSet<Integer> neededNodes = new HashSet<Integer>();
					neededNodes.add(ge.nodes[1]);
					DijkstraProjection dp2 = new DijkstraProjection(g,
							ge.nodes[0], null, neededNodes);
					GXYTEdge dpPrior = dp2.getEdge(ge.nodes[1]);
					double trace = (dpPrior == null) ? Double.MAX_VALUE
							: LinAlg.trace(dpPrior.P);

					if (dpPrior != null) {
						MultiGaussian mg = new MultiGaussian(dpPrior.P,
								dpPrior.z);
						double mahl = mg.getMahalanobisDistance(ge.z);
						if (mahl > maxMahal) {
							continue;
						}
					}

					if (trace > besttrace) {
						bestedge = ge;
						besttrace = trace;
					}
				}

				if (bestedge != null) {
					// hack to prevent very close edges being added
					// if(Math.abs(bestedge.nodes[0]-bestedge.nodes[1])>6){
					g.edges.add(bestedge);
					synchronized (addedEdges) {
						addedEdges.add(bestedge);
					}
					acceptedEdges.remove(bestedge);
					// }
				}
			}
		}
	}

	// depth: current depth (index to pidxs and eidxs that we'll write to)
	void recurse(int depth, int pidxs[], HashSet<GXYTEdge> visitedEdges,
			double xyt[], ArrayList<GXYTEdge> acceptedEdges) {
		if (depth >= pidxs.length)
			return;

		ArrayList<GXYTEdge> edges = hypoNodes.get(pidxs[depth - 1]);
		if (edges == null)
			return;

		// limit branching factor to 10.
		int deidx = 1; // Math.max(1, edges.size() / 10);

		for (int eidx = 0; eidx < edges.size(); eidx += deidx) {
			GXYTEdge ge = edges.get(eidx);

			if (visitedEdges.contains(ge))
				continue;

			// follow this edge, flipping it if it's the wrong direction.
			double newxyt[];

			if (ge.nodes[0] == pidxs[depth - 1]) {
				newxyt = LinAlg.xytMultiply(xyt, ge.z);
				pidxs[depth] = ge.nodes[1];
			} else {
				newxyt = LinAlg.xytMultiply(xyt, LinAlg.xytInverse(ge.z));
				pidxs[depth] = ge.nodes[0];
			}

			// have we arrived back where we started?
			if (pidxs[depth] == pidxs[0]) {

				if (depth != 3)
					continue;

				// We've formed a loop. Is it any good?
				final double xyerr = Math.sqrt(newxyt[0] * newxyt[0]
						+ newxyt[1] * newxyt[1]);
				final double terr = Math.abs(MathUtil.mod2pi(newxyt[2]));

				// accuracy of loop
				final double xythresh = 0.05;
				final double tthresh = Math.toRadians(1);

				if (xyerr < xythresh && terr < tthresh) {

					// we got some confirmation! Mark all these
					// edges as having been validated. Each edge
					// in the loop "votes" for all the other edges
					// in the loop

					// did we add one or more edges from this loop to the
					// real graph?
					boolean added = false;
					for (GXYTEdge te : visitedEdges) {
						HashSet<GXYTEdge> affirmingEdges = (HashSet<GXYTEdge>) te
								.getAttribute("affirming-edges");
						if (affirmingEdges == null) {
							affirmingEdges = new HashSet<GXYTEdge>();
							te.setAttribute("affirming-edges", affirmingEdges);
						}

						for (GXYTEdge se : visitedEdges) {
							if (se == te)
								continue;
							affirmingEdges.add(se);
						}

						// is this edge now worth adding to the real graph?
						if (te.getAttribute("added") != null)
							continue;

						if (true || affirmingEdges.size() >= 4) {
							acceptedEdges.add(te);

							added = true;
							te.setAttribute("added", "yes");
						}
					}

				}
			} else {
				// It's not a loop.

				// Don't allow loops which use very similar edges
				// (i.e., a robot is stationary and generates two
				// identical (but potentially incorrect) edges.
				boolean reject = false;

				if (true) {
					GNode ga = g.nodes.get(pidxs[depth]);
					// int ga_robot_id = (Integer)
					// ga.getAttribute("robot_id");

					for (int i = 0; i < depth - 1; i++) {
						GNode gb = g.nodes.get(pidxs[i]);

						// if (ga_robot_id == (Integer)
						// gb.getAttribute("robot_id")) {
						if (true) {
							double xyt_local_a[] = LinAlg.copy(ga.state);
							double xyt_local_b[] = LinAlg.copy(gb.state);

							double xydist = Math
									.sqrt(LinAlg.sq(xyt_local_a[0]
											- xyt_local_b[0])
											+ LinAlg.sq(xyt_local_a[1]
													- xyt_local_b[1]));
							double tdist;
							if (ga instanceof GXYTNode
									&& gb instanceof GXYTNode) {
								tdist = Math.abs(MathUtil.mod2pi(xyt_local_a[2]
										- xyt_local_b[2]));
							} else {
								tdist = 0;
							}

							if (xydist < 0.25 && tdist < Math.toRadians(30))
								reject = true;
						}
					}
				}

				if (reject)
					continue;

				// Avoid redundant loops by selecting only the loops in
				// which the largest pose index is the first one
				if (pidxs[depth] > pidxs[depth - 1])
					continue;

				// recurse.
				visitedEdges.add(ge);
				recurse(depth + 1, pidxs, visitedEdges, newxyt, acceptedEdges);
				visitedEdges.remove(ge);

			}
		}
	}

	double mahalanobisDistance(GXYTEdge ge, double range) {
		// which nodes *might* be in the neighborhood?
		// Compute the mahalanobis distance that the nodes are
		// within 5 meters of each other.

		// how far away is the other node according to the dijkstra edge?
		double dist = Math.sqrt(ge.z[0] * ge.z[0] + ge.z[1] * ge.z[1]);

		// when we subtract our sensor range, what fraction of the distance do
		// we have left?
		double dist_ratio;

		if (dist < range)
			dist_ratio = 0;
		else
			dist_ratio = (dist - range) / dist;

		// compute the positional error, accounting for our sensor range.
		double err[] = new double[] { ge.z[0] * dist_ratio,
				ge.z[1] * dist_ratio };

		// compute mahalanobis distance of err.
		double W[][] = ge.getW();

		// non invertible. Usually means that we're querying the
		// distance of node 'a' with respect to the same node 'a'.
		if (W == null) {
			if (dist < range)
				return 0;
			else
				return Double.MAX_VALUE;
		}

		return Math.sqrt(err[0] * err[0] * W[0][0] + 2 * err[0] * err[1]
				* W[0][1] + err[1] * err[1] * W[1][1]);
	}

	void optimizeFull(Graph g) {
		synchronized (g) {
			double chi2Before = g.getErrorStats().chi2normalized;
			CholeskySolver gs = new CholeskySolver(g,
					new MinimumDegreeOrdering());
			gs.verbose = false;

			for (int iter = 0; iter < 10; iter++) {
				gs.iterate();

				double chi2After = g.getErrorStats().chi2normalized;

				if (chi2After >= 0.8 * chi2Before || chi2After < 0.00001)
					break;

				chi2Before = chi2After;
			}
		}
	}
}
