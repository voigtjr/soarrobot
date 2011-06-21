package edu.umich.robot.slam;

import java.util.ArrayList;

import april.config.Config;
import april.graph.GXYTEdge;
import april.graph.GXYTNode;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.laser.scanmatcher.HillClimbing;
import april.laser.scanmatcher.MultiResolutionMatcher;
import april.util.GridMap;

public class LoopClosureMatcher
{
	//config / grip map for the loop closure matcher
	Config config;
    GridMap gmLC;
    
    //following parameters are initialized to default values
    //pass in true values via 'config' file (see below)
    double metersPerPixel = 0.05;
    double rangeCovariance = 0.1;
    double gridmap_size = 40;
    double distErr = 0.15;
    double rotErr = 0.1;
    double minMatchScore = 20*255;
    double trange = Math.toRadians(10);
    double trangeres = Math.toRadians(1);
    double xyrange = 1.5;
    double xCov = 1.0;
    double yCov = 1.0;
    double tCov = Math.toRadians(5.0);

    //loop closure scan matcher
    MultiResolutionMatcher clMatcher;

    public LoopClosureMatcher(Config config2)
    {
        //set the config file for loop closures
    	config = config2;
        this.metersPerPixel = this.config.getDouble("metersPerPixel", metersPerPixel);
        this.rangeCovariance = this.config.getDouble("rangeCovariance", rangeCovariance);
        this.gridmap_size = this.config.getDouble("gridmap_size", gridmap_size);
        this.distErr = this.config.getDouble("distErr", distErr);
        this.rotErr = this.config.getDouble("rotErr", rotErr);
        this.minMatchScore = this.config.getDouble("minMatchScore", minMatchScore);
        this.trange = Math.toRadians(this.config.getDouble("trange", Math.toDegrees(trange)));
        this.trangeres = Math.toRadians(this.config.getDouble("trangeres", Math.toDegrees(trangeres)));
        this.xyrange = this.config.getDouble("xyrange", xyrange);
        this.minMatchScore = this.config.getDouble("minMatchScore", minMatchScore);
        this.xCov = this.config.getDouble("xCov", xCov);
        this.yCov = this.config.getDouble("yCov", yCov);
        this.tCov = this.config.getDouble("tCov", tCov);
        
        //loop closure scan matcher
        clMatcher = new MultiResolutionMatcher();
    }

    //initializes the grip map for the loop closure scan matcher (must be invoked prior to calling match)
    public void init(ArrayList<double[]> ptsA)
    {
        this.gmLC = ScanUtils.MakeModel(ptsA,-(gridmap_size)/2,-(gridmap_size)/2,gridmap_size,gridmap_size,metersPerPixel,rangeCovariance);
        clMatcher.setModel(this.gmLC);
    }
    
    public double getSearchDist()
    {
    	return this.xyrange;
    }

    //set the grip map for the loop closure scan matcher
    public void setModel(GridMap gmap)
    {
        this.gmLC = gmap;
        clMatcher.setModel(this.gmLC);
    }

    //get the grip map for the loop closure scan matcher
    public GridMap getGridMap()
    {
        return this.gmLC;
    }

    /*
     * The match function returns an edge with the correct
     * RBT transformation but just a constant covariance
     * The parameters for the constant covariance can be 
     * assigned within the 'config' file. 
     */
    public GXYTEdge match(GXYTNode a,GXYTNode b)
    {
        double[] prior = new double[] {0.0,0.0,0.0};
        return match(a,b,prior);
    }

    public GXYTEdge match(GXYTNode a,GXYTNode b, double[] priorXYT)
    {
    	//new edge to return if successful match found
        GXYTEdge newEdge = new GXYTEdge();

        //nodeA is the reference scan and nodeB is the scan being matched to
        double[] poseA = a.state;
        double[] poseB = b.state;
        ArrayList<double[]> ptsB = (ArrayList<double[]>) b.getAttribute("points");
        
        double ddist = LinAlg.distance(poseA, poseB, 2);
        double dtheta = Math.abs(MathUtil.mod2pi(poseA[2] - poseB[2]));

        double P[][] = LinAlg.diag(new double[] { LinAlg.sq(ddist*distErr)+0.01,
                			LinAlg.sq(ddist*distErr)+0.01,
                			LinAlg.sq(dtheta*rotErr)+Math.toRadians(1)});
        
        double priorScale = 1.0 / (ptsB.size()*30.0);
        double priorScaled[][] = LinAlg.scale(P, priorScale);        

        double r[] = clMatcher.match(ptsB, priorXYT, LinAlg.inverse(priorScaled),
                                     priorXYT,
                                     xyrange, xyrange, trange, trangeres);

        HillClimbing hc = new HillClimbing(new Config());
        hc.setModel(gmLC);

        r = hc.match(ptsB, priorXYT, priorScaled, LinAlg.copy(r,3));

        if(r[3]<minMatchScore){ //score too low to add an edge
            return null;
        }
        
        /*Additional constraints to consider . . . 
          		double xytb_posterior[] = LinAlg.copy(res, 3);
                double dxyt_posterior[] = LinAlg.xytInvMul31(xyta, xytb_posterior);

                double err[] = LinAlg.subtract(xytb_posterior, xytb);
                err[2] = MathUtil.mod2pi(err[2]);

                double err_dist = Math.sqrt(LinAlg.sq(err[0]) + LinAlg.sq(err[1]));
                double err_t = Math.abs(MathUtil.mod2pi(err[2]));

                if (err_dist > 0.5 || err_t > Math.toRadians(15)) {
                    sweep.xyt_slam = LinAlg.xytMultiply(sweepa.xyt_slam, dxyt_prior);
                    System.out.println("rejecting slam");
                } else {
                    sweep.xyt_slam = LinAlg.xytMultiply(sweepa.xyt_slam, dxyt_posterior);
                }
            }

         */

        newEdge.z = new double[]{r[0],r[1],r[2]};
        newEdge.P = LinAlg.diag(new double[]{xCov, yCov, tCov});
        newEdge.setAttribute("Score", r[3]);
        return newEdge;
    }
}