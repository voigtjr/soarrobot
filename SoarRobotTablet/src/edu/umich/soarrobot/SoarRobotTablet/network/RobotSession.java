package edu.umich.soarrobot.SoarRobotTablet.network;

import java.io.IOException;
import java.util.HashMap;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import android.graphics.PointF;
import april.lcmtypes.pose_t;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class RobotSession extends Thread {

	private static final long FPS = 60;
	private static final long DELAY = 1000 / FPS;
	
	SoarRobotTablet activity;
	String server;
	int port;
	boolean running;
		
	// Used to keep track of the robot
	HashMap<String, Integer> channelsToIds;
	
	public RobotSession(SoarRobotTablet activity, String server, int port) {
		this.activity = activity;
		this.server = server;
		this.port = port;
		channelsToIds = new HashMap<String, Integer>();
		
		subscriber = new LCMSubscriber() {
			@Override
			public void messageReceived(LCM lcm, String channel,
					LCMDataInputStream ins) {
				try {
					pose_t p = new pose_t(ins);
					PointF robotLocation = new PointF((float)p.pos[0], (float)p.pos[1]);
					
					String channelName = "ROBOT";
					int objectID = channelsToIds.get(channelName);
					SimObject obj = RobotSession.this.activity.getMapView().getObject(objectID);

					if (obj != null) {
						obj.setLocation(robotLocation);
					}

				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		};
		
		try {
			LCM lcm = new LCM("udp://141.212.203.213:12122");
			lcm.subscribe("POSE_seek", subscriber);
		} catch (IOException e) {
			e.printStackTrace();
		}
	
	}
	
	LCMSubscriber subscriber;
	
	@Override
	public void run() {
		running = true;
		try {
			activity.showAlert("Connecting to server at " + server + ":" + port);
			
			// simulate delay from connecting to server
			try {
				sleep(3000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			
			
			// simulate object classes gathered from new connection
			HashMap<String, HashMap<String, Object>> classes = new HashMap<String, HashMap<String, Object>>();
			
			HashMap<String, Object> blocks = new HashMap<String, Object>();
			blocks.put("color", "green");
			blocks.put("size", new float[]{0.5f, 0.5f});
			classes.put("block", blocks);
			
			HashMap<String, Object> robots = new HashMap<String, Object>();
			robots.put("color", "blue");
			robots.put("size", new float[]{1.0f, 1.0f});
			classes.put("robot", robots);
			
			SimObject.init(classes);
			
			MapView map = activity.getMapView();
			map.addObject(new SimObject("block", new PointF(2.0f, 1.5f)));
			map.addObject(new SimObject("block", new PointF(3.0f, 3.0f)));
			map.addObject(new SimObject("block", new PointF(4.0f, 4.5f)));
			
			SimObject robot = new SimObject("robot", new PointF(0.0f, 0.0f));
			channelsToIds.put("ROBOT", robot.getID());
			map.addObject(robot);
			
			activity.showAlert("Connected to server at " + server + ":" + port);

		} catch (Exception e) {
			e.printStackTrace();
		}
		
		// main run loop
		/*
		while (running) {
			
			// simulate incoming udp message
			String channelName = "ROBOT";
			int objectID = channelsToIds.get(channelName);
			SimObject obj = activity.getMapView().getObject(objectID);

			if (obj != null) {
				// Vary its location a little bit
				PointF p = obj.getLocation();
				p.x += 0.05f;
				p.y += 0.03f;
				p.x = p.x % 10.0f;
				p.y = p.y % 10.0f;
				obj.setLocation(p);
			}
			
			try {
				sleep(DELAY);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
		*/
		
	}
	
	public void end() {
		running = false;
	}
}
