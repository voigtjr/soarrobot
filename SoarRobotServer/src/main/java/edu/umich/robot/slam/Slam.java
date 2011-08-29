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

	// SLAM setup
	// use odometry sensors
	boolean useOdom = true;
	// attempt to close the loop
	boolean closeLoop = true;
	// artificially inflate the odometry information with error (used for
	// simulation environment)
	boolean noisyOdom = true;
	// attempt to locate doors and add them to the pose graph
	boolean includeDoors = true;

	// configuration files for parameters within the slam process, the open loop
	// scan matching process, the close loop scan matching process, and the door
	// finding process
	Config slamConfig;
	Config loopmatchConfig;
	Config loopcloseConfig;
	Config doorfinderConfig;

	// door finder instance
	DoorFinder doorFinder;
	// tracks the index within graph.nodes of the last pose (used by the door
	// finder class)
	int lastPoseIndex = 0;

	// open loop scan matcher if not using odometry
	MultiResolutionScanMatcher openMatch;
	double metersPerPixel = 0.05;
	double rangeCovariance = 0.1;
	double search_x_m = 0.2;
	double search_y_m = 0.2;
	double search_theta_rad = Math.toRadians(15);
	double search_theta_res_rad = Math.toRadians(1);
	double gridmapsize = 20;
	int old_scan_decay = 5;
	GridMap gm;

	// error parameters for sampling from noisy odometry
	Random r = new Random();
	double[] al = new double[] { 0.005, 0.005, Math.toRadians(0.05) };

	// array lists to track the ground truth odometry, noise inflated odometry,
	// and SLAM corrected odometry
	ArrayList<double[]> gTruth = new ArrayList<double[]>();
	ArrayList<double[]> pureOdom = new ArrayList<double[]>();
	ArrayList<double[]> slamPose = new ArrayList<double[]>();

	// starter variable to ensure an odometry pose is added to our graph prior
	// to using open loop scan matching
	int starter = 0;

	/**
	 * Loop closure SLAM process parameters. Adjust based on desired
	 * performance, environment, and robot setup.
	 * 
	 * @param matchScore
	 *            Minimum score that must be achieved by the multi-resolution
	 *            scan matcher before an matching edge is returned.
	 * @param thetaRange
	 *            Rotation search space (in radians).
	 * @param thetaRangeRes
	 *            Rotation search space resolution (in radians).
	 * @param searchRange
	 *            Search window size (in meters).
	 * 
	 */
	double matchScore = 8000;
	double thetaRange = Math.toRadians(10);
	double thetaRangeRes = Math.toRadians(1);
	double searchRange = 10;

	// covariance estimate on open loop odometry
	double distErr = 0.15;
	double rotErr = 0.1;

	// loop closure instantiation
	boolean returner;
	LoopClosure LC;

	// matcher for loop closing
	LoopClosureMatcher loopmatcher;

	// graph object
	Graph g = new Graph();

	// scan information
	int maxScanHistory = 5;
	int decimate = 1;
	public int decimateCounter;
	ArrayList<double[]> lastScan;

	// graph parameters
	double pose_dist_thresh_m = 1.0;
	double pose_theta_thresh_rad = Math.toRadians(25);

	// current robot location in global frame
	double xyt[] = new double[3];
	// last ground truth location in global frame
	double groundTruth[] = new double[3];
	// XYT to process odometry as rigid body transforms
	double XYT[] = new double[3];
	// holds additional added edges for visualization
	ArrayList<GEdge> addedEdges = new ArrayList<GEdge>();

	// laser scan class used by open loop scan matcher when not using odometry
	// sensors
	public static class Scan {
		// x, y, t of robot @ time of scan
		double xyt[];

		// 2D points in GLOBAL frame
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
			loopmatchConfig = config.getChild("LoopMatcher");
			loopcloseConfig = config.getChild("LoopCloser");
			doorfinderConfig = config.getChild("DoorFinder");

			// setting clam configuration parameters
			maxScanHistory = slamConfig.getInt("max_scan_history",
					maxScanHistory);
			decimate = slamConfig.getInt("decimate", decimate);
			pose_dist_thresh_m = slamConfig.getDouble("pose_dist_thresh_m",
					pose_dist_thresh_m);
			pose_theta_thresh_rad = Math.toRadians(slamConfig.getDouble(
					"pose_dist_theta_thresh_rad",
					Math.toDegrees(pose_theta_thresh_rad)));
			distErr = slamConfig.getDouble("distErr", distErr);
			rotErr = slamConfig.getDouble("rotErr", rotErr);
			useOdom = slamConfig.getBoolean("useOdom", useOdom);
			closeLoop = slamConfig.getBoolean("closeLoop", closeLoop);
			noisyOdom = slamConfig.getBoolean("noisyOdom", noisyOdom);
			includeDoors = slamConfig.getBoolean("includeDoors", includeDoors);
			matchScore = slamConfig.getDouble("matchScore", matchScore);
			thetaRange = Math.toRadians(slamConfig.getDouble("thetaRange",
					thetaRange));
			thetaRangeRes = Math.toRadians(slamConfig.getDouble(
					"thetaRangeRes", thetaRangeRes));
			searchRange = slamConfig.getDouble("searchRange", searchRange);
			metersPerPixel = slamConfig.getDouble("metersPerPixel",
					metersPerPixel);
			rangeCovariance = slamConfig.getDouble("rangeCovariance",
					rangeCovariance);
			search_x_m = slamConfig.getDouble("search_x_m", search_x_m);
			search_y_m = slamConfig.getDouble("search_y_m", search_y_m);
			search_theta_rad = Math.toRadians(slamConfig.getDouble(
					"search_theta_rad", search_theta_rad));
			search_theta_res_rad = Math.toRadians(slamConfig.getDouble(
					"search_theta_res_rad", search_theta_res_rad));
			gridmapsize = slamConfig.getDouble("gridmapsize", gridmapsize);
			old_scan_decay = slamConfig
					.getInt("old_scan_decay", old_scan_decay);
		}

		// Initializations based on slam setup
		if (closeLoop)
			LC = new LoopClosure(loopcloseConfig);
		if (!useOdom) {
			openMatch = new MultiResolutionScanMatcher(new Config());
			gm = GridMap.makeMeters(-(gridmapsize / 2), -(gridmapsize / 2),
					gridmapsize, gridmapsize, metersPerPixel, 0);
			starter = 1;
		}
		if (includeDoors) {
			doorFinder = new DoorFinder(doorfinderConfig);
		}
		this.g = new Graph();
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
			synchronized (gTruth) {
				gTruth.add(LinAlg.copy(odomxyt));
			}
			synchronized (pureOdom) {
				pureOdom.add(LinAlg.copy(odomxyt));
			}
			synchronized (slamPose) {
				slamPose.add(LinAlg.copy(odomxyt));
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
				synchronized (gTruth) {
					gTruth.add(LinAlg.copy(odomxyt));
				}

				// true movement RBT parameters
				double[] priorU = LinAlg.xytInvMul31(groundTruth, odomxyt);
				priorU[2] = MathUtil.mod2pi(priorU[2]);
				groundTruth = LinAlg.copy(odomxyt);

				// sampling from noise parameters, assuming error of al[num] per
				// each full unit of true movement
				double s1 = Math.abs(al[0] * priorU[0]);
				double s2 = Math.abs(al[1] * priorU[1]);
				double s3 = Math.abs(al[2] * priorU[2]);

				double Dx = priorU[0] + Math.sqrt(s1) * r.nextGaussian();
				double Dy = priorU[1] + Math.sqrt(s2) * r.nextGaussian();
				double Dt = priorU[2] + Math.sqrt(s3) * r.nextGaussian();

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
					search_x_m, search_y_m, search_theta_rad,
					search_theta_res_rad);
			double[] positionUpdate = posterior.getMean();
			xyt = LinAlg.copy(positionUpdate);
		}

		// where was our last pose
		GXYTNode NA = (GXYTNode) g.nodes.get(lastPoseIndex);
		double ddist = LinAlg.distance(xyt, NA.state, 2);
		double dtheta = Math.abs(MathUtil.mod2pi(xyt[2] - NA.state[2]));

		if (ddist > pose_dist_thresh_m || dtheta > pose_theta_thresh_rad) {

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
			loopmatcher = new LoopClosureMatcher(loopmatchConfig);

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
			loopmatcher.setMatchParameters(matchScore, thetaRange,
					thetaRangeRes, searchRange);

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
				LC.close();
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
		synchronized (slamPose) {
			slamPose.add(LinAlg.copy(xyt));
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

		gm.recenter((minx + maxx) / 2, (miny + maxy) / 2, 5);

		gm.subtract(old_scan_decay);

		ArrayList<double[]> gpoints = s.gpoints;
		double[] last_lp = new double[1];
		double lastpidx = 0;
		;
		for (int pidx = 0; pidx < gpoints.size(); pidx++) {
			double lp[] = gpoints.get(pidx);
			gm.setValue(lp[0], lp[1], (byte) 255);

			if (last_lp != null && lastpidx == pidx - 1) {
				double dist = LinAlg.distance(lp, last_lp);
				if (dist < 0.25) {
					gm.drawLine(last_lp[0], last_lp[1], lp[0], lp[1],
							(byte) 255);
				}
			}

			last_lp = lp;
			lastpidx = pidx;
		}

		float f[] = SigProc.makeGaussianFilter(1.5, 5);
		f = LinAlg.scale(f, 1.0 / LinAlg.max(f));
		gm.filterFactoredCenteredMax(f, f);

		openMatch.setModel(gm);
	}

	class LoopClosure {
		int autoclose_lastnode = 2; // last node that we processed for loop
									// closures
		int autoclose_lastedge = 0; // last edge that we checked for being
									// odometry
		int max_loop_length = 4;
		int maxScanMatchAttempts = 40;
		int maxAcceptEdges = 10; // maximum number of edges to add per node
		double minTrace = 0.2;
		int maxHypothesisAge = 20;
		double mahlMax = 1.0; // reject hypotheses
		double hypoAddDistThres = 5.0;
		double closeFrequency = 1;

		// hypotheses grouped according to the nodes they
		// connect. Each edge appears twice (once for each node it
		// connects). Includes odometry edges.
		HashMap<Integer, ArrayList<GXYTEdge>> nodeHypotheses = new HashMap<Integer, ArrayList<GXYTEdge>>();

		public LoopClosure() {
			this(null);
		}

		public LoopClosure(Config config) {
			if (config != null) {
				max_loop_length = config.getInt("max_loop_length",
						max_loop_length);
				maxScanMatchAttempts = config.getInt("maxScanMatchAttempts",
						maxScanMatchAttempts);
				maxAcceptEdges = config
						.getInt("maxAcceptEdges", maxAcceptEdges);
				minTrace = config.getDouble("minTrace", minTrace);
				maxHypothesisAge = config.getInt("maxHypothesisAge",
						maxHypothesisAge);
				mahlMax = config.getDouble("mahlMax", mahlMax);
				hypoAddDistThres = config.getDouble("hypoAddDistThres",
						hypoAddDistThres);
				closeFrequency = config.getDouble("closeFrequency",
						closeFrequency);
			}
		}

		void addHypothesis(GXYTEdge ge) {
			ArrayList<GXYTEdge> a = nodeHypotheses.get(ge.nodes[0]);
			if (a == null) {
				a = new ArrayList<GXYTEdge>();
				nodeHypotheses.put(ge.nodes[0], a);
			}
			a.add(ge);

			ArrayList<GXYTEdge> b = nodeHypotheses.get(ge.nodes[1]);
			if (b == null) {
				b = new ArrayList<GXYTEdge>();
				nodeHypotheses.put(ge.nodes[1], b);
			}
			b.add(ge);
		}

		// Do NOT thread this
		public void close() {
			autoclose_lastnode = lastPoseIndex;
			// nothing to do
			if (includeDoors) {
				if (g == null || (g.nodes.size() - doorFinder.numDoors) <= 2) {
					return;
				}
				if (autoclose_lastnode == g.nodes.size()) {
					return;
				}
			} else {
				if (g == null || g.nodes.size() <= 2) {
					return;
				}
				if (autoclose_lastnode == g.nodes.size()) {
					return;
				}
			}

			// add other XYT edges to our hypothesis pool
			synchronized (g) {
				for (; autoclose_lastedge < g.edges.size(); autoclose_lastedge++) {
					GEdge _ge = g.edges.get(autoclose_lastedge);
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
			GXYTNode refnode = (GXYTNode) g.nodes.get(autoclose_lastnode);

			ArrayList<GXYTEdge> candidates = new ArrayList<GXYTEdge>();

			synchronized (g) {
				dp = new DijkstraProjection(g,
						(Integer) refnode.getAttribute("node_index"));
			}

			// frequency of loop closing on nodes
			// will attempt to close very Nth node added to the graph
			if (autoclose_lastnode % closeFrequency == 0) {

				// identify the set of nodes that we'd like to do scanmatching
				// to.
				for (int i = 0; i < autoclose_lastnode; i++) {
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
					double mahl = mahalanobisDistance(dpPrior, hypoAddDistThres);
					if (mahl < 0.45)
						candidates.add(dpPrior);
				}

				// shuffle, shuffle, shuffle...
				Collections.shuffle(candidates);

				for (int cidx = 0; cidx < Math.min(maxScanMatchAttempts,
						candidates.size()); cidx++) {
					GXYTEdge dpPrior = candidates.get(cidx);

					// attempting to match nodes
					GXYTNode nodeA = (GXYTNode) g.nodes.get(dpPrior.nodes[0]);
					GXYTNode nodeB = (GXYTNode) g.nodes.get(dpPrior.nodes[1]);
					double[] prior = LinAlg.xytInvMul31(nodeA.state,
							nodeB.state);
					prior[2] = MathUtil.mod2pi(prior[2]);
					GXYTEdge ge = loopmatcher.match(nodeA, nodeB, prior);

					if (ge != null) {
						ge.z[2] = MathUtil.mod2pi(ge.z[2]);
						ge.nodes = new int[] { dpPrior.nodes[0],
								dpPrior.nodes[1] };
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
				int pidxs[] = new int[max_loop_length];
				pidxs[0] = autoclose_lastnode;

				ArrayList<GXYTEdge> acceptedEdges = new ArrayList<GXYTEdge>();

				recurse(1, pidxs, visitedEdges, xytl, acceptedEdges);

				synchronized (g) {

					for (int i = 0; i < maxAcceptEdges; i++) {
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
								if (mahl > mahlMax) {
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
			autoclose_lastnode++;
		}

		// depth: current depth (index to pidxs and eidxs that we'll write to)
		void recurse(int depth, int pidxs[], HashSet<GXYTEdge> visitedEdges,
				double xyt[], ArrayList<GXYTEdge> acceptedEdges) {
			if (depth >= pidxs.length)
				return;

			ArrayList<GXYTEdge> edges = nodeHypotheses.get(pidxs[depth - 1]);
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
								te.setAttribute("affirming-edges",
										affirmingEdges);
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

								double xydist = Math.sqrt(LinAlg
										.sq(xyt_local_a[0] - xyt_local_b[0])
										+ LinAlg.sq(xyt_local_a[1]
												- xyt_local_b[1]));
								double tdist;
								if (ga instanceof GXYTNode
										&& gb instanceof GXYTNode) {
									tdist = Math.abs(MathUtil
											.mod2pi(xyt_local_a[2]
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
					recurse(depth + 1, pidxs, visitedEdges, newxyt,
							acceptedEdges);
					visitedEdges.remove(ge);

				}
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
