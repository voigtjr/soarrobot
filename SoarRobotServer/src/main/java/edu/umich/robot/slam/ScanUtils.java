package edu.umich.robot.slam;

import java.util.*;

import april.image.SigProc;
import april.jmat.*;
import april.util.*;

public class ScanUtils
{
	//Define from 'config' parameters within SLAM routine
    public static GridMap MakeModel(ArrayList<double []> gpoints,double minX,double minY,double sizeX,double sizeY,double metersPerPixel,double rangeCovariance)
    {
    	//FOR USE WITH OLDER IMPLEMENTATION
        //ArrayList<ArrayList<double[]>> gcontours;
        //ContourExtractor contourExtractor = new ContourExtractor();
        GridMap gmnew = GridMap.makeMeters(minX,minY,sizeX,sizeY,metersPerPixel,0);

        double minx = Double.MAX_VALUE, maxx = -Double.MAX_VALUE;
        double miny = Double.MAX_VALUE, maxy = -Double.MAX_VALUE;

        // Compute bounds of the scans.
        for (double p[] : gpoints) {
            minx = Math.min(minx, p[0]);
            maxx = Math.max(maxx, p[0]);
            miny = Math.min(miny, p[1]);
            maxy = Math.max(maxy, p[1]);
        }

        gmnew.recenter((minx+maxx)/2, (miny+maxy)/2, 5);
        
        /*The following are previous gripmap initialization methods. 
         *They have been commented out due to poor results.
         *Use at own risk.  
         */
        
        //GridMap.LUT lut = gmnew.makeGaussianLUT(1.0, 0, 1.0 / rangeCovariance);
        //gcontours = contourExtractor.getContours(gpoints);

        //for (ArrayList<double[]> c : gcontours) {
            //for (int i = 0; i+1 < c.size(); i++) {
                //double p0[] = c.get(i);
                //double p1[] = c.get(i+1);

                //double length = LinAlg.distance(p0, p1);

                //gmnew.drawRectangle((p0[0]+p1[0])/2, (p0[1]+p1[1])/2, length, 0,
                                 //Math.atan2(p1[1]-p0[1], p1[0]-p0[0]),
                                 //lut);
            //}
        //}
        
        
        //for(double[] point:gpoints){
            //gmnew.drawDot(point[0], point[1], lut);
        //}
        
        //Current Successful Implementation (Similar to MAGIC Method for gridMap Rendering)
        double[] last_lp = new double[1];
        double lastpidx = 0;;
        for(int pidx = 0; pidx <gpoints.size(); pidx ++){
        	double lp[] = gpoints.get(pidx);
        	gmnew.setValue(lp[0], lp[1], (byte) 255);
        
        	if(last_lp != null && lastpidx == pidx-1){
        		double dist = LinAlg.distance(lp, last_lp);
        		if(dist < 0.25)	{
        			gmnew.drawLine(last_lp[0], last_lp[1], lp[0], lp[1], (byte) 255);
        		}
        	}
        
        	last_lp = lp;
        	lastpidx = pidx;
        }
        
        float f[] = SigProc.makeGaussianFilter(1.5, 5);
        f = LinAlg.scale(f, 1.0 / LinAlg.max(f));
        gmnew.filterFactoredCenteredMax(f, f);

        return gmnew;
    }
    

}
