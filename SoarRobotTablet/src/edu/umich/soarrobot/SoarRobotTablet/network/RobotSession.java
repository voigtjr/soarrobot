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
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class RobotSession extends Thread {
	
	SoarRobotTablet activity;
	String server;
	int port;
	
	Socket tcpClient;
	PrintWriter tcpWriter;
	Scanner tcpScanner;
		
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
			LCM lcm = new LCM("udp://" + server + ":" + port);
			lcm.subscribe("POSE_seek", subscriber);
		} catch (IOException e) {
			e.printStackTrace();
		}
		
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
		
		activity.showAlert("Connected to server at " + server + ":" + port, Toast.LENGTH_LONG);
	}
	
	LCMSubscriber subscriber;

	public String sendMessage(String message) {
		tcpWriter.println(message);
		tcpWriter.flush();
		String response = tcpScanner.next();
		return response;
	}
}
