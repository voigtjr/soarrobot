package edu.umich.robot.slam;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;

import april.config.Config;
import april.graph.CholeskySolver;
import april.graph.DijkstraProjection;
import april.graph.GEdge;
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
import april.lcmtypes.laser_t;
import april.lcmtypes.pose_t;
import april.util.GridMap;
import april.util.Tic;

public class SoarSlam
{
    // slam setup
    boolean useOdom = false;
    boolean closeLoop = true;

    // config files, one for slam process, loopmatch process, and loopclosure
    // process
    Config slamConfig;
    Config loopmatchConfig;
    Config loopcloseConfig;

    // variables for 3d plotting
    Tic tic;
    double initialT = 0.0;

    // data parameters
    double minLaser = 0.5;
    double maxLaser = 8;
    int starter = 0;

    // covariance estimate
    double distErr = 0.15;
    double rotErr = 0.1;

    // loop closure information
    boolean returner;
    LoopClosure LC;

    // matcher for loop closing
    LoopClosureMatcher loopmatcher;

    // hypoth edges

    // graph objects
    Graph g = new Graph();
    GridMap gm;

    // open loop scan objects
    MultiResolutionScanMatcher matcher;

    // scan information
    double metersPerPixel = 0.05;
    double rangeCovariance = 0.1;
    int maxScanHistory = 5;
    int decimate = 1;

    // search parameters for correlative scan matcher
    double search_x_m = 0.2;
    double search_y_m = 0.2;
    double search_theta_rad = Math.toRadians(15);
    double search_theta_res_rad = Math.toRadians(1);

    // graph parameters
    double pose_dist_thresh_m = 1.0;
    double pose_theta_thresh_rad = Math.toRadians(25);

    // gridmap size in meters
    double gridmap_size = 40;

    // scan decay for gridmap
    int old_scan_decay = 5;
    public int decimateCounter;

    // list of all scans
    ArrayList<Scan> scans = new ArrayList<Scan>();
    // current robot location in global frame
    double xyt[] = new double[3];
    // XYT to process odometry as rigid body transforms
    double XYT[] = new double[3];
    // holds additional added edges for visualization
    ArrayList<GXYTEdge> hypoth = new ArrayList<GXYTEdge>();

    // laser scan class
    public static class Scan
    {
        // x, y, t of robot @ time of scan
        double xyt[];

        // 2D points in GLOBAL frame
        ArrayList<double[]> gpoints;
    }

    public SoarSlam()
    {
        this(null);
    }

    public SoarSlam(Config config)
    {
        // parsing config file
        if (config != null)
        {
            slamConfig = config.getChild("Slam");
        }
        if (slamConfig != null)
        {
            loopmatchConfig = config.getChild("LoopMatcher");
            loopcloseConfig = config.getChild("LoopCloser");
        }

        // setting slam parameters
        if (slamConfig != null)
        {
            metersPerPixel = slamConfig.getDouble("meters_per_pixel", metersPerPixel);
            gridmap_size = slamConfig.getDouble("gridmap_size", gridmap_size);

            // used for rendering
            rangeCovariance = slamConfig.getDouble("range_covariance", rangeCovariance);
            
            maxScanHistory = slamConfig.getInt("max_scan_history", maxScanHistory);
            decimate = slamConfig.getInt("decimate", decimate);
            search_x_m = slamConfig.getDouble("search_x_m", search_x_m);
            search_y_m = slamConfig.getDouble("search_y_m", search_y_m);
            search_theta_rad = Math.toRadians(slamConfig.getDouble("search_theta_rad", Math.toDegrees(search_theta_rad)));
            search_theta_res_rad = Math.toRadians(slamConfig.getDouble("search_theta_res_rad", Math.toDegrees(search_theta_res_rad)));
            old_scan_decay = slamConfig.getInt("old_scan_decay", old_scan_decay);
            pose_dist_thresh_m = slamConfig.getDouble("pose_dist_thresh_m", pose_dist_thresh_m);
            pose_theta_thresh_rad = Math.toRadians(slamConfig.getDouble("pose_dist_theta_thresh_rad", Math.toDegrees(pose_theta_thresh_rad)));
            minLaser = slamConfig.getDouble("minLaser", minLaser);
            maxLaser = slamConfig.getDouble("maxLaser", maxLaser);
            distErr = slamConfig.getDouble("distErr", distErr);
            rotErr = slamConfig.getDouble("rotErr", rotErr);
            useOdom = slamConfig.getBoolean("useOdom", useOdom);
            closeLoop = slamConfig.getBoolean("closeLoop", closeLoop);
        }

        // Initializations
        matcher = new MultiResolutionScanMatcher(new Config());
        LC = new LoopClosure(loopcloseConfig);
        gm = GridMap.makeMeters(-(gridmap_size / 2), -(gridmap_size / 2), gridmap_size, gridmap_size, metersPerPixel, 0);
        g = new Graph();
        tic = new Tic();
        loopmatcher = new LoopClosureMatcher(loopmatchConfig);
        if (!useOdom) starter = 1;
    }

    /** Pass in pose_t lcm type **/
    public void processOdometry(pose_t curPose)
    {
        // if we aren't using the odometry, then return
        if (!useOdom) return;

        // grab the pose
        double[] odomxyt = curPose.pos;

        // map the current angle
        odomxyt[2] = MathUtil.mod2pi(odomxyt[2]);

        // allow for an initial offset position
        if (starter == 0)
        {
            xyt = LinAlg.copy(odomxyt);
            XYT = LinAlg.copy(odomxyt);
            starter++;
            return;
        }

        // RBT update (u is 'T'), XYT --> A, estPose --> B
        double[] u = LinAlg.xytInvMul31(XYT, odomxyt);

        // Reassign XYT values
        XYT = LinAlg.copy(odomxyt);

        // simple updating of the predicted pose
        xyt = LinAlg.xytMultiply(xyt, u);
        xyt[2] = MathUtil.mod2pi(xyt[2]);
    }

    /** Pass in laser_t lcm type **/
    public void processScan(laser_t lidarScan)
    {
        // wait for first position message to arrive before beginning scanning
        if (starter == 0) return;

        // process this scan into points
        ArrayList<double[]> rpoints = new ArrayList<double[]>();
        double rad0 = lidarScan.rad0;
        for (int i = 0; i < lidarScan.nranges; i++)
        {
            double c = Math.cos(rad0);
            double s = Math.sin(rad0);
            rad0 += lidarScan.radstep;

            // Thresholding errors
            if (lidarScan.ranges[i] > minLaser && lidarScan.ranges[i] < maxLaser)
            {
                double[] xy = { lidarScan.ranges[i] * c, lidarScan.ranges[i] * s };
                rpoints.add(xy);
            }
        }

        // first scan ever
        if ((scans.size() == 0))
        {
            // our first-ever scan.
            GXYTNode gn = new GXYTNode();
            gn.state = xyt;
            gn.init = xyt;

            GXYTPosEdge edge = new GXYTPosEdge();
            edge.nodes = new int[] { 0 };
            edge.z = LinAlg.copy(gn.state);
            edge.P = LinAlg.diag(new double[] { 0.01, 0.01, 0.001 });
            g.edges.add(edge);

            initialT = tic.toc();
            gn.setAttribute("points", new ArrayList<double[]>(rpoints), new april.util.PointArrayCoder());
            gn.setAttribute("time", tic.toc() - initialT);
            gn.setAttribute("node_index", g.nodes.size());
            g.nodes.add(gn);

            Scan scan = new Scan();
            scan.xyt = xyt;
            scan.gpoints = LinAlg.transform(xyt, rpoints);
            scans.add(scan);

            drawScan(scan);

            return;
        }

        // throwing away scan based upon parameters in declaration above
        decimateCounter++;
        if (decimateCounter != decimate) return;
        decimateCounter = 0;

        // scan matcher using prior scans
        MultiGaussian posterior = matcher.match(rpoints, xyt, null, search_x_m, search_y_m, search_theta_rad, search_theta_res_rad);
        double[] positionUpdate = posterior.getMean();

        if (!useOdom) xyt = LinAlg.copy(positionUpdate);

        // where was our last scan?
        double lastxyt[] = scans.get(scans.size() - 1).xyt;
        double ddist = LinAlg.distance(xyt, lastxyt, 2);
        double dtheta = Math.abs(MathUtil.mod2pi(xyt[2] - lastxyt[2]));

        if (ddist > pose_dist_thresh_m || dtheta > pose_theta_thresh_rad)
        {

            if (useOdom) xyt = LinAlg.copy(positionUpdate);

            GXYTNode gn = new GXYTNode();
            gn.state = xyt;
            gn.init = xyt;

            gn.setAttribute("points", new ArrayList<double[]>(rpoints), new april.util.PointArrayCoder());
            gn.setAttribute("time", tic.toc() - initialT);
            gn.setAttribute("node_index", g.nodes.size());
            g.nodes.add(gn);

            if (scans.size() > maxScanHistory) scans.remove(0);

            GXYTEdge ge = new GXYTEdge();
            ge.z = LinAlg.xytInvMul31(lastxyt, xyt);

            // this is our movement covariance estimate
            ge.P = LinAlg.diag(new double[] { LinAlg.sq(ddist * distErr) + 0.01, LinAlg.sq(ddist * distErr) + 0.01,
                    LinAlg.sq(dtheta * rotErr) + Math.toRadians(1) });
            ge.nodes = new int[2];
            ge.nodes[0] = g.nodes.size() - 2;
            ge.nodes[1] = g.nodes.size() - 1;

            g.edges.add(ge);

            // loopclosure
            if (closeLoop)
            {
                int curEdges = g.edges.size();
                while (true)
                {
                    LC.run(0);
                    if (returner == true)
                    {
                        returner = false;
                        break;
                    }
                }
                if (curEdges < g.edges.size())
                {
                    optimizeFull(g);
                    xyt = LinAlg.copy(g.nodes.get(g.nodes.size() - 1).state);
                }
            }

            // updating the open loop gridmap
            Scan scan = new Scan();
            scan.xyt = xyt;
            scan.gpoints = LinAlg.transform(xyt, rpoints);
            scans.add(scan);
            drawScan(scan);
        }
    }

    void drawScan(Scan s)
    {
        double minx = Double.MAX_VALUE, maxx = -Double.MAX_VALUE;
        double miny = Double.MAX_VALUE, maxy = -Double.MAX_VALUE;

        // Compute bounds of the scans.
        for (double p[] : s.gpoints)
        {
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
        for (int pidx = 0; pidx < gpoints.size(); pidx++)
        {
            double lp[] = gpoints.get(pidx);
            gm.setValue(lp[0], lp[1], (byte) 255);

            if (last_lp != null && lastpidx == pidx - 1)
            {
                double dist = LinAlg.distance(lp, last_lp);
                if (dist < 0.25)
                {
                    gm.drawLine(last_lp[0], last_lp[1], lp[0], lp[1], (byte) 255);
                }
            }

            last_lp = lp;
            lastpidx = pidx;
        }

        float f[] = SigProc.makeGaussianFilter(1.5, 5);
        f = LinAlg.scale(f, 1.0 / LinAlg.max(f));
        gm.filterFactoredCenteredMax(f, f);

        matcher.setModel(gm);
    }

    class LoopClosure
    {
        int autoclose_lastnode = 0; // last node that we processed for loop
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
        double closeFrequency = 5;

        // hypotheses grouped according to the nodes they
        // connect. Each edge appears twice (once for each node it
        // connects). Includes odometry edges.
        HashMap<Integer, ArrayList<GXYTEdge>> nodeHypotheses = new HashMap<Integer, ArrayList<GXYTEdge>>();

        public LoopClosure()
        {
            this(null);
        }

        public LoopClosure(Config config)
        {
            if (config != null)
            {
                max_loop_length = config.getInt("max_loop_length", max_loop_length);
                maxScanMatchAttempts = config.getInt("maxScanMatchAttempts", maxScanMatchAttempts);
                maxAcceptEdges = config.getInt("maxAcceptEdges", maxAcceptEdges);
                minTrace = config.getDouble("minTrace", minTrace);
                maxHypothesisAge = config.getInt("maxHypothesisAge", maxHypothesisAge);
                mahlMax = config.getDouble("mahlMax", mahlMax);
                hypoAddDistThres = config.getDouble("hypoAddDistThres", hypoAddDistThres);
                closeFrequency = config.getDouble("closeFrequency", closeFrequency);
            }
        }

        void addHypothesis(GXYTEdge ge)
        {
            ArrayList<GXYTEdge> a = nodeHypotheses.get(ge.nodes[0]);
            if (a == null)
            {
                a = new ArrayList<GXYTEdge>();
                nodeHypotheses.put(ge.nodes[0], a);
            }
            a.add(ge);

            ArrayList<GXYTEdge> b = nodeHypotheses.get(ge.nodes[1]);
            if (b == null)
            {
                b = new ArrayList<GXYTEdge>();
                nodeHypotheses.put(ge.nodes[1], b);
            }
            b.add(ge);
        }

        // not 'really' setup to be threaded...so don't do it!
        public void run(double dt)
        {
            // nothing to do
            if (g == null || g.nodes.size() <= 2)
            {
                returner = true;
                return;
            }
            if (autoclose_lastnode == g.nodes.size())
            {
                returner = true;
                return;
            }

            // add other XYT edges to our hypothesis pool
            synchronized (g)
            {
                for (; autoclose_lastedge < g.edges.size(); autoclose_lastedge++)
                {
                    GEdge _ge = g.edges.get(autoclose_lastedge);
                    String type = (String) _ge.getAttribute("type");
                    if (type == null) type = "odom";

                    if (!type.equals("auto") && (_ge instanceof GXYTEdge))
                    {
                        GXYTEdge ge = (GXYTEdge) _ge;
                        addHypothesis(ge);
                    }
                }
            }

            DijkstraProjection dp;

            // generate edges between node "autoclose_last" and earlier nodes.
            GXYTNode refnode = (GXYTNode) g.nodes.get(autoclose_lastnode);

            ArrayList<GXYTEdge> candidates = new ArrayList<GXYTEdge>();

            synchronized (g)
            {
                dp = new DijkstraProjection(g, (Integer) refnode.getAttribute("node_index"));
            }

            // frequency of loop closing on nodes
            if (autoclose_lastnode % closeFrequency == 0)
            {

                // loop closure match to be used below in matching
                loopmatcher = new LoopClosureMatcher(loopmatchConfig);
                ArrayList<double[]> ptsA = (ArrayList<double[]>) refnode.getAttribute("points");
                loopmatcher.init(ptsA);

                // identify the set of nodes that we'd like to do scanmatching
                // to.
                for (int i = 0; i < autoclose_lastnode; i++)
                {
                    GXYTEdge dpPrior = dp.getEdge(i);

                    // double trace = LinAlg.trace(dpPrior.P);

                    // which nodes *might* be in the neighborhood?
                    // Compute the mahalanobis distance that the nodes are
                    // within 5 meters of each other.

                    // HACK - add additional structure to requirements of
                    // possible candidates:
                    // forcing the candidates to be within a "true" distance of
                    // the maximum
                    // xy search area of the loop closure matcher parameter
                    double curDist = LinAlg.distance(refnode.state, g.nodes.get(dpPrior.nodes[1]).state);
                    if (curDist > loopmatcher.getSearchDist()) continue;

                    double mahl = mahalanobisDistance(dpPrior, hypoAddDistThres); // hypoAddDistThres
                                                                                  // =
                                                                                  // dist
                                                                                  // (2-5
                                                                                  // m
                                                                                  // )
                    if (mahl < 0.45) candidates.add(dpPrior);
                }

                Collections.shuffle(candidates);

                for (int cidx = 0; cidx < Math.min(maxScanMatchAttempts, candidates.size()); cidx++)
                {
                    GXYTEdge dpPrior = candidates.get(cidx);
                    int i = dpPrior.nodes[1];

                    // if (LinAlg.trace(dpPrior.P) < minTrace)
                    // continue;

                    // try to match
                    // avoid over confidence by adding fudge
                    // dpPrior.P[0][0] += LinAlg.sq(.5);
                    // dpPrior.P[1][1] += LinAlg.sq(.5);
                    // dpPrior.P[2][2] += LinAlg.sq(Math.toRadians(5));

                    GXYTNode nodeA = (GXYTNode) g.nodes.get(dpPrior.nodes[0]);
                    GXYTNode nodeB = (GXYTNode) g.nodes.get(dpPrior.nodes[1]);
                    double[] prior = LinAlg.xytInvMul31(nodeA.state, nodeB.state);
                    prior[2] = MathUtil.mod2pi(prior[2]);
                    GXYTEdge ge = loopmatcher.match(nodeA, nodeB, prior);

                    if (ge != null)
                    {
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
                int pidxs[] = new int[max_loop_length];
                pidxs[0] = autoclose_lastnode;

                ArrayList<GXYTEdge> acceptedEdges = new ArrayList<GXYTEdge>();

                recurse(1, pidxs, visitedEdges, xytl, acceptedEdges);

                synchronized (g)
                {

                    for (int i = 0; i < maxAcceptEdges; i++)
                    {
                        GXYTEdge bestedge = null;
                        double besttrace = minTrace;

                        for (GXYTEdge ge : acceptedEdges)
                        {
                            HashSet<Integer> neededNodes = new HashSet<Integer>();
                            neededNodes.add(ge.nodes[1]);
                            DijkstraProjection dp2 = new DijkstraProjection(g, ge.nodes[0], null, neededNodes);
                            GXYTEdge dpPrior = dp2.getEdge(ge.nodes[1]);
                            double trace = (dpPrior == null) ? Double.MAX_VALUE : LinAlg.trace(dpPrior.P);

                            if (dpPrior != null)
                            {
                                MultiGaussian mg = new MultiGaussian(dpPrior.P, dpPrior.z);
                                double mahl = mg.getMahalanobisDistance(ge.z);
                                if (mahl > mahlMax)
                                {
                                    continue;
                                }
                            }

                            if (trace > besttrace)
                            {
                                bestedge = ge;
                                besttrace = trace;
                            }
                        }

                        if (bestedge != null)
                        {
                            // hack to prevent very close edges being added
                            // if(Math.abs(bestedge.nodes[0]-bestedge.nodes[1])>6){
                            g.edges.add(bestedge);
                            hypoth.add(bestedge);
                            acceptedEdges.remove(bestedge);
                            // }
                        }
                    }

                    // Optimize Now
                    optimizeFull(g);
                }
            }
            autoclose_lastnode++;
        }

        // depth: current depth (index to pidxs and eidxs that we'll write to)
        void recurse(int depth, int pidxs[], HashSet<GXYTEdge> visitedEdges, double xyt[], ArrayList<GXYTEdge> acceptedEdges)
        {
            if (depth >= pidxs.length) return;

            ArrayList<GXYTEdge> edges = nodeHypotheses.get(pidxs[depth - 1]);
            if (edges == null) return;

            // limit branching factor to 10.
            int deidx = 1; // Math.max(1, edges.size() / 10);

            for (int eidx = 0; eidx < edges.size(); eidx += deidx)
            {
                GXYTEdge ge = edges.get(eidx);

                if (visitedEdges.contains(ge)) continue;

                // follow this edge, flipping it if it's the wrong direction.
                double newxyt[];

                if (ge.nodes[0] == pidxs[depth - 1])
                {
                    newxyt = LinAlg.xytMultiply(xyt, ge.z);
                    pidxs[depth] = ge.nodes[1];
                }
                else
                {
                    newxyt = LinAlg.xytMultiply(xyt, LinAlg.xytInverse(ge.z));
                    pidxs[depth] = ge.nodes[0];
                }

                // have we arrived back where we started?
                if (pidxs[depth] == pidxs[0])
                {

                    if (depth != 3) continue;

                    // We've formed a loop. Is it any good?
                    final double xyerr = Math.sqrt(newxyt[0] * newxyt[0] + newxyt[1] * newxyt[1]);
                    final double terr = Math.abs(MathUtil.mod2pi(newxyt[2]));

                    // accuracy of loop
                    final double xythresh = 0.05;
                    final double tthresh = Math.toRadians(1);

                    if (xyerr < xythresh && terr < tthresh)
                    {

                        // we got some confirmation! Mark all these
                        // edges as having been validated. Each edge
                        // in the loop "votes" for all the other edges
                        // in the loop

                        // did we add one or more edges from this loop to the
                        // real graph?
                        boolean added = false;
                        for (GXYTEdge te : visitedEdges)
                        {
                            HashSet<GXYTEdge> affirmingEdges = (HashSet<GXYTEdge>) te.getAttribute("affirming-edges");
                            if (affirmingEdges == null)
                            {
                                affirmingEdges = new HashSet<GXYTEdge>();
                                te.setAttribute("affirming-edges", affirmingEdges);
                            }

                            for (GXYTEdge se : visitedEdges)
                            {
                                if (se == te) continue;
                                affirmingEdges.add(se);
                            }

                            // is this edge now worth adding to the real graph?
                            if (te.getAttribute("added") != null) continue;

                            if (true || affirmingEdges.size() >= 4)
                            {
                                acceptedEdges.add(te);

                                added = true;
                                te.setAttribute("added", "yes");
                            }
                        }

                    }
                }
                else
                {
                    // It's not a loop.

                    // Don't allow loops which use very similar edges
                    // (i.e., a robot is stationary and generates two
                    // identical (but potentially incorrect) edges.
                    boolean reject = false;

                    if (true)
                    {
                        GXYTNode ga = (GXYTNode) g.nodes.get(pidxs[depth]);
                        // int ga_robot_id = (Integer)
                        // ga.getAttribute("robot_id");

                        for (int i = 0; i < depth - 1; i++)
                        {
                            GXYTNode gb = (GXYTNode) g.nodes.get(pidxs[i]);

                            // if (ga_robot_id == (Integer)
                            // gb.getAttribute("robot_id")) {
                            if (true)
                            {
                                double xyt_local_a[] = LinAlg.copy(ga.state);
                                double xyt_local_b[] = LinAlg.copy(gb.state);

                                double xydist = Math.sqrt(LinAlg.sq(xyt_local_a[0] - xyt_local_b[0]) + LinAlg.sq(xyt_local_a[1] - xyt_local_b[1]));
                                double tdist = Math.abs(MathUtil.mod2pi(xyt_local_a[2] - xyt_local_b[2]));

                                if (xydist < 0.25 && tdist < Math.toRadians(30)) reject = true;
                            }
                        }
                    }

                    if (reject) continue;

                    // Avoid redundant loops by selecting only the loops in
                    // which the largest pose index is the first one
                    if (pidxs[depth] > pidxs[depth - 1]) continue;

                    // recurse.
                    visitedEdges.add(ge);
                    recurse(depth + 1, pidxs, visitedEdges, newxyt, acceptedEdges);
                    visitedEdges.remove(ge);

                }
            }
        }

    }

    double mahalanobisDistance(GXYTEdge ge, double range)
    {
        // which nodes *might* be in the neighborhood?
        // Compute the mahalanobis distance that the nodes are
        // within 5 meters of each other.

        // how far away is the other node according to the dijkstra edge?
        double dist = Math.sqrt(ge.z[0] * ge.z[0] + ge.z[1] * ge.z[1]);

        // when we subtract our sensor range, what fraction of the distance do
        // we have left?
        double dist_ratio;

        if (dist < range) dist_ratio = 0;
        else dist_ratio = (dist - range) / dist;

        // compute the positional error, accounting for our sensor range.
        double err[] = new double[] { ge.z[0] * dist_ratio, ge.z[1] * dist_ratio };

        // compute mahalanobis distance of err.
        double W[][] = ge.getW();

        // non invertible. Usually means that we're querying the
        // distance of node 'a' with respect to the same node 'a'.
        if (W == null)
        {
            if (dist < range) return 0;
            else return Double.MAX_VALUE;
        }

        return Math.sqrt(err[0] * err[0] * W[0][0] + 2 * err[0] * err[1] * W[0][1] + err[1] * err[1] * W[1][1]);
    }

    void optimizeFull(Graph g)
    {
        synchronized (g)
        {
            double chi2Before = g.getErrorStats().chi2normalized;

            CholeskySolver gs = new CholeskySolver(g, new MinimumDegreeOrdering());
            gs.verbose = false;

            for (int iter = 0; iter < 10; iter++)
            {
                gs.iterate();

                double chi2After = g.getErrorStats().chi2normalized;

                if (chi2After >= 0.8 * chi2Before || chi2After < 0.00001) break;

                chi2Before = chi2After;
            }
        }
    }
}
