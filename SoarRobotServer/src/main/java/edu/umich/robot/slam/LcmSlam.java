package edu.umich.robot.slam;

import java.awt.BorderLayout;
import java.awt.Color;
import java.util.ArrayList;

import javax.swing.JFrame;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import april.config.Config;
import april.config.ConfigUtil;
import april.graph.GXYTEdge;
import april.graph.GXYTNode;
import april.graph.Graph;
import april.jmat.LinAlg;
import april.lcmtypes.laser_t;
import april.lcmtypes.pose_t;
import april.util.ParameterGUI;
import april.util.PeriodicTasks;
import april.vis.VisCanvas;
import april.vis.VisChain;
import april.vis.VisData;
import april.vis.VisDataLineStyle;
import april.vis.VisDataPointStyle;
import april.vis.VisRobot;
import april.vis.VisWorld;

public class LcmSlam implements LCMSubscriber
{	
	//config files
	Config lcmConfig;
	
	//three d viewing
	int viewCount = 0;
	
	//soarslam instance
	SoarSlam slammer;
	
	//objects for threading
    PeriodicTasks dispPeriodicTasks = new PeriodicTasks();
    DisplayTask dispTask = new DisplayTask();
	
    //lcm parameters
    LCM myLCM;
    String laserName = "hoku";
    String odomName = "odom";
    
    //laser_t available for plotting the scan
    laser_t cScan;

    //vis objects
    VisWorld vw = new VisWorld();
    VisWorld vw2 = new VisWorld();
    VisCanvas vc = new VisCanvas(vw);
    VisCanvas vc2 = new VisCanvas(vw2);
    ParameterGUI pg = new ParameterGUI();
    JFrame jf = new JFrame("Map View");
    JFrame jf2 = new JFrame("Scan View");

    public static void main(String args[]){
        Config config = ConfigUtil.getDefaultConfig(args);
    	LcmSlam lc = new LcmSlam(config);
    	lc.myLCM = LCM.getSingleton();
        lc.myLCM.subscribe(lc.laserName, lc);
        lc.myLCM.subscribe(lc.odomName, lc);
        
        while(true){
        	try {
        		Thread.sleep(1000);
        	} 
        	catch (Exception ex) {
        		System.out.print("\nException caught in Main\n");
        		ex.printStackTrace();
        	}
        }
    }
    
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
    	try {
    		if(channel.startsWith(laserName)){
    			laser_t laser = new laser_t(ins);
    			cScan = laser;
    			slammer.processScan(laser);
    		}
    		
    		if(channel.startsWith(odomName)){
    			pose_t pose = new pose_t(ins);
    			slammer.processOdometry(pose);
    		}
    	}
    	catch (Exception ex) {
    		System.out.println("Error Reading the message: "+ex);
    		ex.printStackTrace();
    	}
    }

    public LcmSlam(Config config)
    {
        //parsing config file
    	lcmConfig = config.getChild("lcm");
    	this.laserName = lcmConfig.getString("laserName", laserName);
    	this.odomName = lcmConfig.getString("odomName", odomName);
    	this.slammer = new SoarSlam(config);
        
    	//vis objects
    	pg.addCheckBoxes("hypothesis", "Show Added Edges", true);
    	pg.addCheckBoxes("threed", "View 3-D", false);
    	pg.addCheckBoxes("showscan", "Show Scan", false);
    	pg.addCheckBoxes("showcurpose", "Show Current Pose", true);
        
        //main frame
        vc.getViewManager().setInterfaceMode(3);
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(785, 785);
        jf.setVisible(true);
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        
        //scan frame
        vc2.getViewManager().setInterfaceMode(3);
        jf2.setLayout(new BorderLayout());
        jf2.add(vc2, BorderLayout.CENTER);
        jf2.setSize(485, 485);
        if(pg.gb("showscan"))
        	jf2.setVisible(true);
        if(!pg.gb("showscan"))
        	jf2.setVisible(false);
    	
    	//thread map vis
    	dispPeriodicTasks.addFixedDelay(dispTask, 0.2);
    	dispPeriodicTasks.setRunning(true);
    }
    
    class DisplayTask implements PeriodicTasks.Task{
        public void run(double dt){
        	displayGraph(slammer.g, vw);
        }
    }
    
    void displayGraph(Graph g, VisWorld visual)
    {
        VisWorld.Buffer vb = visual.getBuffer("worldstate");
        VisWorld.Buffer vRobot = visual.getBuffer("robots");
        
        //plotting current pose
        if(pg.gb("showcurpose")){
        	double[] pose = slammer.xyt;
        	vRobot.addBuffered(new VisChain(LinAlg.xytToMatrix(pose), new VisRobot(Color.yellow)));
        }
        
        //showing scan
		if(pg.gb("showscan")){
			jf2.setVisible(true);
			//process this scan into points
	    	ArrayList<double[]> rpoints = new ArrayList<double[]>();
			double rad0=cScan.rad0;
			for(int i=0;i<cScan.nranges;i++){
				double c = Math.cos(rad0);
				double s = Math.sin(rad0);
				rad0+=cScan.radstep;

				//Thresholding errors
				if(cScan.ranges[i]>slammer.minLaser && cScan.ranges[i]<slammer.maxLaser){
					double[] xy = {cScan.ranges[i]*c,cScan.ranges[i]*s};
					rpoints.add(xy);
				}
			}
        	VisWorld.Buffer vb2 = vw2.getBuffer("worldstate");
            vb2.addBuffered(new VisChain(LinAlg.xytToMatrix(new double[]{0,0,0}), new VisRobot(Color.red)));
            vb2.addBuffered(new VisData(new VisDataPointStyle(Color.blue, 2), rpoints));
            vb2.switchBuffer();
        }
		else{
			jf2.setVisible(false);
		}

        //plotting all nodes and their corresponding scans
        for (int i = 0; i < g.nodes.size(); i++) {
        	
        	//node's state
        	double[] pose = g.nodes.get(i).state;
        	double nodeTime = (Double)g.nodes.get(i).getAttribute("time")*0.25;
        	
        	//xyzrpy
        	double[] xyzrpy = new double[] {pose[0], pose[1], nodeTime, 0, 0, pose[2]};
            
        	//node's scan
            ArrayList<double[]> LPoints = (ArrayList<double[]>)g.nodes.get(i).getAttribute("points");
            ArrayList<double[]> GPoints = LinAlg.transform(pose, LPoints);

            //add pose to buffer
            if(!pg.gb("threed"))
            	vRobot.addBuffered(new VisChain(LinAlg.xytToMatrix(pose), new VisRobot(Color.red)));
            if(pg.gb("threed")){
            	vRobot.addBuffered(new VisChain(xyzrpy, new VisRobot(Color.red)));
            }


            //add scan to buffer
            vb.addBuffered(new VisData(new VisDataPointStyle(Color.blue, 2), GPoints));
        }
        
        //Displaying added edges from loop closure task
        if(pg.gb("hypothesis")){
        	for(GXYTEdge gh : slammer.hypoth){
        		VisData vd = new VisData(new VisDataPointStyle(Color.orange, 3),
        				new VisDataLineStyle(Color.green, 1));
        
        			GXYTNode ga = (GXYTNode) g.nodes.get(gh.nodes[0]);
        			GXYTNode gb = (GXYTNode) g.nodes.get(gh.nodes[1]);
        			double ta = (Double) g.nodes.get(gh.nodes[0]).getAttribute("time")*0.25;
        			double tb = (Double) g.nodes.get(gh.nodes[1]).getAttribute("time")*0.25;
        			
        			if(!pg.gb("threed")){
        				vd.add(new double[] {ga.state[0], ga.state[1]});
        				vd.add(new double[] {gb.state[0], gb.state[1]});
        			}
        			
        			
        			if(pg.gb("threed")){
        				vd.add(new double[] {ga.state[0], ga.state[1], ta});
        				vd.add(new double[] {gb.state[0], gb.state[1], tb});        				
        			}
        			vb.addBuffered(vd);
        	}
        }
        
        //three d viewer (rotating view .. needs tuning depending on current data set...)
        if(pg.gb("threed") && slammer.starter!=0){
        	double ang = Math.toRadians(viewCount);
			double[] cameraPos = new double[] {15*Math.cos(ang), 15*Math.sin(ang), 10};
			double[] viewPos = new double[] {g.nodes.get(0).state[0], g.nodes.get(0).state[1], 0};
			double[] up = new double[] {0, 0, 1};
			vc.getViewManager().viewGoal.lookAt(cameraPos, viewPos, up);
			viewCount++;  
        }
        
        //Update vis
        vb.switchBuffer();
        vRobot.switchBuffer();
    }
    
}