package edu.umich.soarrobot.SoarRobotTablet.network;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.Scanner;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import android.graphics.PointF;
import android.widget.Toast;
import april.lcmtypes.pose_t;
import april.jmat.LinAlg;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class RobotSession extends Thread implements LCMSubscriber {
	
	SoarRobotTablet activity;
	String server;
	int port;
	
	Socket tcpClient;
	PrintWriter tcpWriter;
	Scanner tcpScanner;
	
	LCM lcm;
	
	public RobotSession(SoarRobotTablet activity, String server, int port) {
		this.activity = activity;
		this.server = server;
		this.port = port;
		
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
		
		sendMessage("hello");
		
		try {
			// This needs to be the client address.
			String clientHost = tcpClient.getLocalAddress().toString();
			String clientPort = "" + tcpClient.getLocalPort();
			lcm = new LCM("udp:/" + clientHost + ":" + clientPort);
			lcm.subscribe("POSE_seek", this);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		activity.showAlert("Connected to server at " + server + ":" + port, Toast.LENGTH_LONG);
		
		// Initialize class definitions
		HashMap<String, HashMap<String, Object>> classes = new HashMap<String, HashMap<String,Object>>();
		HashMap<String, Object> robot = new HashMap<String, Object>();
		classes.put("robot", robot);
		SimObject.init(classes);
		
		// Add robot to the map
		activity.getMapView().addObject(new SimObject("robot", new PointF(0.0f, 0.0f)));
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
			double[] robotOrientation = new double[4];
			for (int i=0; i < robotOrientation.length; i++) {
			    robotOrientation[i] = p.orientation[i];
			}
			
			// Assume the robot gets ID 0
			SimObject obj = activity.getMapView().getObject(0);
			
			if (obj != null) {
				obj.setLocation(robotLocation);
				obj.setOrientation(LinAlg.quatToRollPitchYaw(robotOrientation));
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}
}