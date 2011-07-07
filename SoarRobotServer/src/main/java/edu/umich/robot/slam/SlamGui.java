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

public class SlamGui implements LCMSubscriber {
	// SoarSlam instance
	Slam slam;

	// Objects for threading
	PeriodicTasks dispPeriodicTasks = new PeriodicTasks();
	Task dispTask = new Task() {
		public void run(double dt) {
			displayGraph(mapFrame.getWorld());
		}
	};

	// Vis objects
	SlamMapFrame mapFrame = new SlamMapFrame("Map View");
	SlamScanFrame scanFrame = new SlamScanFrame("Scan View");
	ParameterGUI parameterGui = mapFrame.getParameterGui();

	// For use by lcm-listening GUIs.
	String laserName;
	String odomName;

	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
		try {
			if (channel.startsWith(laserName)) {
				laser_t laser = new laser_t(ins);
				ArrayList<double[]> rpoints = new ArrayList<double[]>();
				double rad0 = laser.rad0;
				int skipBeams = 1; // allows for the skipping of beam returns
				for (int i = 0; i < laser.nranges; i++) {
					double c = Math.cos(rad0);
					double s = Math.sin(rad0);
					rad0 += laser.radstep;

					if (laser.ranges[i] > 10)
						continue;
					// TODO check laser range before adding point to rpoints
					if (i % skipBeams == 0) {
						double[] xy = { laser.ranges[i] * c,
								laser.ranges[i] * s };
						rpoints.add(xy);
					}
				}
				slam.processScan(rpoints);
			}

			if (channel.startsWith(odomName)) {
				pose_t pose = new pose_t(ins);
				// pass to "processOdometry" robots [x,y,theta] pose in global
				// coordinate frame
				double[] q = pose.orientation;
				double y = 2 * ((q[0] * q[3]) + (q[1] * q[2]));
				double x = 1 - 2 * (Math.pow(q[2], 2) + Math.pow(q[3], 2));
				double angle = Math.atan2(y, x);
				double[] odomxyt = new double[] { pose.pos[0], pose.pos[1],
						angle };
				slam.processOdometry(odomxyt);
			}
		} catch (Exception ex) {
			System.out.println("Error Reading the message: " + ex);
			ex.printStackTrace();
		}
	}

	public SlamGui() {
		this(null);
	}

	/**
	 * Creates a SlamGui without subscribing to any LCM channels
	 */
	public SlamGui(Config config) {
		if (config != null) {
			slam = new Slam(config);
		} else {
			slam = new Slam();
		}
		if (parameterGui.gb("showscan"))
			scanFrame.setVisible(true);
		if (!parameterGui.gb("showscan"))
			scanFrame.setVisible(false);

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
	public SlamGui(Config config, String laserName, String odomName) {
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

	public Slam getSlam() {
		return slam;
	}

	void displayGraph(VisWorld visual) {
		VisWorld.Buffer worldBuffer = visual.getBuffer("worldstate");
		VisWorld.Buffer robotsBuffer = visual.getBuffer("robots");

		// Plot current pose
		if (parameterGui.gb("showcurpose")) {
			double[] pose = slam.xyt;
			robotsBuffer.addBuffered(new VisChain(LinAlg.xytToMatrix(pose),
					new VisRobot(Color.yellow)));
		}

		// Show scans as they are received
		if (parameterGui.gb("showscan")) {
			scanFrame.setVisible(true);
			// process this scan into points
			ArrayList<double[]> rpoints = new ArrayList<double[]>();
			rpoints = slam.lastScan;
			VisWorld.Buffer vb2 = scanFrame.getWorld().getBuffer("worldstate");
			vb2.addBuffered(new VisChain(LinAlg.xytToMatrix(new double[] { 0,
					0, 0 }), new VisRobot(Color.red)));
			vb2.addBuffered(new VisData(new VisDataPointStyle(Color.blue, 2),
					rpoints));
			vb2.switchBuffer();
		} else {
			scanFrame.setVisible(false);
		}

		// Plot all nodes and their corresponding scans
		for (int i = 0; i < slam.g.nodes.size(); i++) {
			// Node's state
			double[] pose = slam.g.nodes.get(i).state;

			// Node's scan
			@SuppressWarnings("unchecked")
			ArrayList<double[]> LPoints = (ArrayList<double[]>) slam.g.nodes
					.get(i).getAttribute("points");
			ArrayList<double[]> GPoints = LinAlg.transform(pose, LPoints);

			// Add pose to buffer
			robotsBuffer.addBuffered(new VisChain(LinAlg.xytToMatrix(pose),
					new VisRobot(Color.red)));

			// Add scan to buffer
			worldBuffer.addBuffered(new VisData(new VisDataPointStyle(
					Color.blue, 2), GPoints));
		}

		// Display added edges from loop closure task
		if (parameterGui.gb("hypothesis")) {
			synchronized (slam.hypoth) {
				for (GXYTEdge gh : slam.hypoth) {
					VisData vd = new VisData(new VisDataPointStyle(Color.green,
							2), new VisDataLineStyle(Color.green, 1));

					GXYTNode ga = (GXYTNode) slam.g.nodes.get(gh.nodes[0]);
					GXYTNode gb = (GXYTNode) slam.g.nodes.get(gh.nodes[1]);

					vd.add(new double[] { ga.state[0], ga.state[1] });
					vd.add(new double[] { gb.state[0], gb.state[1] });

					worldBuffer.addBuffered(vd);
				}
			}
		}

		// Show additional paths (ground truth:red | pure odometry:black | slam
		// corrected: yellow)
		if (parameterGui.gb("gTruth")) {
			VisData vd = new VisData(new VisDataPointStyle(Color.red, 2),
					new VisDataLineStyle(Color.red, 2));
			synchronized (slam.gTruth) {
				for (double[] gt : slam.gTruth)
					vd.add(new double[] { gt[0], gt[1] });
				double[] grn = slam.gTruth.get(slam.gTruth.size() - 1);
				// System.out.println("Ground Truth Location:" + grn[0] + ", " +
				// grn[1] + ", " + grn[2]);
			}
			worldBuffer.addBuffered(vd);
		}
		if (parameterGui.gb("pureOdom")) {
			VisData vd = new VisData(new VisDataPointStyle(Color.black, 2),
					new VisDataLineStyle(Color.black, 2));
			synchronized (slam.pureOdom) {
				for (double[] po : slam.pureOdom)
					vd.add(new double[] { po[0], po[1] });
				double[] loc = slam.pureOdom.get(slam.pureOdom.size() - 1);
				// System.out.println("Pure Odometry Location:" + loc[0] + ", "
				// + loc[1] + ", " + loc[2]);
			}
			worldBuffer.addBuffered(vd);
		}
		if (parameterGui.gb("slamPose")) {
			VisData vd = new VisData(new VisDataPointStyle(Color.yellow, 2),
					new VisDataLineStyle(Color.yellow, 2));
			synchronized (slam.slamPose) {
				for (double[] sp : slam.slamPose)
					vd.add(new double[] { sp[0], sp[1] });
			}
			worldBuffer.addBuffered(vd);
		}

		// Update vis
		worldBuffer.switchBuffer();
		robotsBuffer.switchBuffer();
	}

	public static void main(String args[]) {
		Config config = ConfigUtil.getDefaultConfig(args);
		new SlamGui(config, "hoku", "odom");
	}

}