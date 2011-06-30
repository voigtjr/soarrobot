package edu.umich.robot.slam;

import java.awt.Color;
import java.util.ArrayList;

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
import april.util.PeriodicTasks.Task;
import april.vis.VisChain;
import april.vis.VisData;
import april.vis.VisDataLineStyle;
import april.vis.VisDataPointStyle;
import april.vis.VisRobot;
import april.vis.VisWorld;

public class SlamGui implements LCMSubscriber
{
    // 3D viewing
    int viewCount = 0;

    // SoarSlam instance
    Slam slam;

    // Objects for threading
    PeriodicTasks dispPeriodicTasks = new PeriodicTasks();
    Task dispTask = new Task()
    {
        public void run(double dt)
        {
            displayGraph(slam.g, mapFrame.getWorld());
        }
    };

    // laser_t available for plotting the scan
    laser_t cScan;

    // Vis objects
    SlamMapFrame mapFrame = new SlamMapFrame("Map View");
    SlamScanFrame scanFrame = new SlamScanFrame("Scan View");
    ParameterGUI parameterGui = mapFrame.getParameterGui();

    // For use by lcm-listening GUIs.
    String laserName;
    String odomName;

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try
        {
            if (channel.startsWith(laserName))
            {
                laser_t laser = new laser_t(ins);
                cScan = laser;
                slam.processScan(laser);
            }

            if (channel.startsWith(odomName))
            {
                pose_t pose = new pose_t(ins);
                slam.processOdometry(pose);
            }
        }
        catch (Exception ex)
        {
            System.out.println("Error Reading the message: " + ex);
            ex.printStackTrace();
        }
    }

    public SlamGui()
    {
        this(null);
    }

    /**
     * Creates a SlamGui without subscribing to any LCM channels
     */
    public SlamGui(Config config)
    {
        if (config != null)
        {
            slam = new Slam(config);
        }
        else
        {
            slam = new Slam();
        }
        if (parameterGui.gb("showscan")) scanFrame.setVisible(true);
        if (!parameterGui.gb("showscan")) scanFrame.setVisible(false);

        // Thread map vis
        dispPeriodicTasks.addFixedDelay(dispTask, 0.2);
        dispPeriodicTasks.setRunning(true);
    }

    /**
     * Creates a SlamGui and subscribes to the specified LCM channels
     * 
     * @param config
     * @param laserName
     * @param odomName
     */
    public SlamGui(Config config, String laserName, String odomName)
    {
        this(config);

        // Parse config file
        Config lcmConfig = config.getChild("lcm");
        this.laserName = lcmConfig.getString("laserName", laserName);
        this.odomName = lcmConfig.getString("odomName", odomName);

        // Subscribe to lcm
        LCM lcm = LCM.getSingleton();
        lcm.subscribe(this.laserName, this);
        lcm.subscribe(this.odomName, this);
    }

    public Slam getSlam()
    {
        return slam;
    }

    void displayGraph(Graph g, VisWorld visual)
    {
        VisWorld.Buffer worldBuffer = visual.getBuffer("worldstate");
        VisWorld.Buffer robotsBuffer = visual.getBuffer("robots");

        // Plot current pose
        if (parameterGui.gb("showcurpose"))
        {
            double[] pose = slam.xyt;
            robotsBuffer.addBuffered(new VisChain(LinAlg.xytToMatrix(pose), new VisRobot(Color.yellow)));
        }

        // Show scan
        if (parameterGui.gb("showscan"))
        {
            scanFrame.setVisible(true);
            // process this scan into points
            ArrayList<double[]> rpoints = new ArrayList<double[]>();
            if (cScan != null)
            {
                double rad0 = cScan.rad0;
                for (int i = 0; i < cScan.nranges; i++)
                {
                    double c = Math.cos(rad0);
                    double s = Math.sin(rad0);
                    rad0 += cScan.radstep;

                    // Thresholding errors
                    if (cScan.ranges[i] > slam.minLaser && cScan.ranges[i] < slam.maxLaser)
                    {
                        double[] xy = { cScan.ranges[i] * c, cScan.ranges[i] * s };
                        rpoints.add(xy);
                    }
                }
            }
            VisWorld.Buffer vb2 = scanFrame.getWorld().getBuffer("worldstate");
            vb2.addBuffered(new VisChain(LinAlg.xytToMatrix(new double[] { 0, 0, 0 }), new VisRobot(Color.red)));
            vb2.addBuffered(new VisData(new VisDataPointStyle(Color.blue, 2), rpoints));
            vb2.switchBuffer();
        }
        else
        {
            scanFrame.setVisible(false);
        }

        // Plot all nodes and their corresponding scans
        for (int i = 0; i < g.nodes.size(); i++)
        {

            // Node's state
            double[] pose = g.nodes.get(i).state;
            double nodeTime = (Double) g.nodes.get(i).getAttribute("time") * 0.25;

            // xyzrpy
            double[] xyzrpy = new double[] { pose[0], pose[1], nodeTime, 0, 0, pose[2] };

            // Node's scan
            @SuppressWarnings("unchecked")
            ArrayList<double[]> LPoints = (ArrayList<double[]>) g.nodes.get(i).getAttribute("points");
            ArrayList<double[]> GPoints = LinAlg.transform(pose, LPoints);

            // Add pose to buffer
            if (!parameterGui.gb("threed")) robotsBuffer.addBuffered(new VisChain(LinAlg.xytToMatrix(pose), new VisRobot(Color.red)));
            if (parameterGui.gb("threed"))
            {
                robotsBuffer.addBuffered(new VisChain(xyzrpy, new VisRobot(Color.red)));
            }

            // Add scan to buffer
            worldBuffer.addBuffered(new VisData(new VisDataPointStyle(Color.blue, 2), GPoints));
        }

        // Display added edges from loop closure task
        if (parameterGui.gb("hypothesis"))
        {
            for (GXYTEdge gh : slam.hypoth)
            {
                VisData vd = new VisData(new VisDataPointStyle(Color.orange, 3), new VisDataLineStyle(Color.green, 1));

                GXYTNode ga = (GXYTNode) g.nodes.get(gh.nodes[0]);
                GXYTNode gb = (GXYTNode) g.nodes.get(gh.nodes[1]);
                double ta = (Double) g.nodes.get(gh.nodes[0]).getAttribute("time") * 0.25;
                double tb = (Double) g.nodes.get(gh.nodes[1]).getAttribute("time") * 0.25;

                if (!parameterGui.gb("threed"))
                {
                    vd.add(new double[] { ga.state[0], ga.state[1] });
                    vd.add(new double[] { gb.state[0], gb.state[1] });
                }

                if (parameterGui.gb("threed"))
                {
                    vd.add(new double[] { ga.state[0], ga.state[1], ta });
                    vd.add(new double[] { gb.state[0], gb.state[1], tb });
                }
                worldBuffer.addBuffered(vd);
            }
        }

        // 3D viewer
        // Rotate view, needs tuning depending on current data set
        if (parameterGui.gb("threed") && slam.starter != 0)
        {
            double ang = Math.toRadians(viewCount);
            double[] cameraPos = new double[] { 15 * Math.cos(ang), 15 * Math.sin(ang), 10 };
            double[] viewPos = new double[] { g.nodes.get(0).state[0], g.nodes.get(0).state[1], 0 };
            double[] up = new double[] { 0, 0, 1 };
            mapFrame.getCanvas().getViewManager().viewGoal.lookAt(cameraPos, viewPos, up);
            viewCount++;
        }

        // Update vis
        worldBuffer.switchBuffer();
        robotsBuffer.switchBuffer();
    }

    public static void main(String args[])
    {
        Config config = ConfigUtil.getDefaultConfig(args);
        new SlamGui(config, "hoku", "odom");
    }

}