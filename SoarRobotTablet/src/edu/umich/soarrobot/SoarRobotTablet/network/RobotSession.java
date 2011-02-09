package edu.umich.soarrobot.SoarRobotTablet.network;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.Scanner;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import android.graphics.PointF;
import android.os.Build;
import android.util.Log;
import android.widget.Toast;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class RobotSession extends Thread implements LCMSubscriber {
	
	SoarRobotTablet activity;
	String server;
	int port;
	boolean paused;
	
	Socket tcpClient;
	PrintWriter tcpWriter;
	Scanner tcpScanner;
	
	LCM lcm;
	String lcmConnectionString;
	ArrayList<String> robotNames;
	
	public RobotSession(SoarRobotTablet activity, String server, int port) {
		this.activity = activity;
		this.server = server;
		this.port = port;
		paused = false;
		
		String deviceName = Build.DEVICE;
		
		try {
			tcpClient = new Socket(server, port);
			tcpWriter = new PrintWriter(tcpClient.getOutputStream());
			tcpScanner = new Scanner(tcpClient.getInputStream()).useDelimiter("\n");
			System.out.println("connected to server, local port: " + tcpClient.getLocalPort());
		} catch (UnknownHostException e) {
			e.printStackTrace();
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		if (tcpClient == null) {
			return;
		}
		
		String map = sendMessage("map");
		String classStrings = sendMessage("classes");
		String robotStrings = sendMessage("robots");
		String objectStrings = sendMessage("objects");
		robotNames = new ArrayList<String>();
		
		// Initialize class definitions
		SimObject.init(classStrings + " splinter { }");
		
		// Add map to the map
		activity.getMapView().deserializeMap(map);
		
		// Add robots to the map
		for (String robotString : robotStrings.split(";")) {
			if (robotString.length() == 0) {
				continue;
			}
			Scanner s = new Scanner(robotString).useDelimiter(" ");
			String name = s.next();
			float x = s.nextFloat();
			float y = -s.nextFloat();
			float theta = s.nextFloat();
			SimObject robot = new SimObject("splinter", new PointF(x, y));
			robot.setTheta(theta);
			robot.setAttribute("name", name);
			activity.getMapView().addRobot(robot);
			robotNames.add(name);
		}
		
		// Add objects to the map
		for (String objString : objectStrings.split(";")) {
			if (objectStrings.length() == 0) {
				continue;
			}
			Scanner s = new Scanner(objString).useDelimiter(" ");
			String name = s.next();
			float x = s.nextFloat();
			float y = -s.nextFloat();
			float theta = s.nextFloat();
			SimObject sim = new SimObject(name, new PointF(x, y));
			sim.setTheta(theta);
			activity.getMapView().addObject(sim);
		}
		
		try {
			// This needs to be the client address.
			String clientHost;
			Log.d("BLAH", "deviceName is " + deviceName);
			if (deviceName.equals("generic")) {
				clientHost = "/10.0.2.15";
			} else {
				clientHost = tcpClient.getLocalAddress().toString();
			}
			
			String clientPort = "" + tcpClient.getLocalPort();
			lcmConnectionString = "udp:/" + clientHost + ":" + clientPort;
			lcm = new LCM(lcmConnectionString);
			for (String robotName : robotNames) {
				lcm.subscribe("POSE_" + robotName, this);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		activity.showAlert("Connected to server at " + server + ":" + port, Toast.LENGTH_LONG);
		activity.getMapView().draw();
	}
	
	public String sendMessage(String message) {
		tcpWriter.println(message);
		tcpWriter.flush();
		String response = tcpScanner.next();
		return response;
	}
	
	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
		try {
			pose_t p = new pose_t(ins);
			PointF robotLocation = new PointF((float)p.pos[0], -(float)p.pos[1]);
			float theta = (float) (LinAlg.quatToRollPitchYaw(p.orientation)[2] * 180.0f / Math.PI);
			SimObject robot = activity.getMapView().getRobot(channel.split("_")[1]);
			if (robot != null) {
				robot.setLocation(robotLocation);
				robot.setTheta(theta);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
		MapView mv = activity.getMapView();
		if (mv != null) {
			mv.draw();
		}
	}
	
	public void pause() {
		paused = true;
		if (lcm != null) {
			lcm.close();
			lcm = null;
		}
	}
	
	public void unPause() {
		paused = false;
		if (lcmConnectionString == null) {
			return;
		}
		try {
			lcm = new LCM(lcmConnectionString);
			for (String robotName : robotNames) {
				lcm.subscribe("POSE_" + robotName, this);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}