/*
 * Copyright (c) 2011, Regents of the University of Michigan
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
package edu.umich.soarrobot.SoarRobotTablet.network;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Scanner;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import android.graphics.PointF;
import android.os.Build;
import android.util.Log;
import android.widget.Toast;
import april.jmat.LinAlg;
import april.lcmtypes.laser_t;
import april.lcmtypes.pose_t;
import april.lcmtypes.waypoint_list_t;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.layout.IMapView;
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

	Object lock = new Object();

	public RobotSession(SoarRobotTablet activity, String server, int port) throws IOException {
		synchronized (lock) {
			this.activity = activity;
			this.server = server;
			this.port = port;
			paused = false;

			String deviceName = Build.DEVICE;

			try {
				tcpClient = new Socket(server, port);
				tcpWriter = new PrintWriter(tcpClient.getOutputStream());
				tcpScanner = new Scanner(tcpClient.getInputStream())
						.useDelimiter("\n");
				System.out.println("connected to server, local port: "
						+ tcpClient.getLocalPort());
			} catch (UnknownHostException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}

			if (tcpClient == null) {
				return;
			}

			robotNames = new ArrayList<String>();
			tcpListener.start();
			try {
				// This needs to be the client address.
				String clientHost;
				Log.d("BLAH", "deviceName is " + deviceName);
				String clientPort = null;
				if (deviceName.equals("generic")) {
					clientHost = "/10.0.2.15";
					clientPort = "12122";
					sendMessage("emulator");
				} else {
					clientHost = tcpClient.getLocalAddress().toString();
					clientPort = "12122"; // clientPort = "" +
											// tcpClient.getLocalPort();
					sendMessage("device");
				}

				lcmConnectionString = "udp:/" + clientHost + ":" + clientPort;
				lcm = new LCM(lcmConnectionString);
				subscribeAllRobots(lcm, robotNames);
			} catch (IOException e) {
				e.printStackTrace();
				throw e;
			}
			sendMessage("map");
			sendMessage("classes");
			sendMessage("robots");
			sendMessage("objects");
			activity.showAlert("Connected to server at " + server + ":" + port,
					Toast.LENGTH_LONG);
			activity.getMapView().draw();
		}
	}

	public void sendMessage(String message) {
		tcpWriter.println(message);
		tcpWriter.flush();
	}

	// Handles a TCP message that was sent from the server.
	private void handleMessage(String message) {
		int space = message.indexOf(' ');
		if (space == -1) {
			return;
		}
		String command = message.substring(0, space);
		if (command.equals("map")) {
			activity.getMapView().deserializeMap(message.substring(space));
		} else if (command.equals("classes")) {
			SimObject.init(message.substring(space) + " splinter { }");
		} else if (command.equals("robots")) {
			for (String robotString : message.substring(space).split(";")) {
				if (robotString.trim().length() == 0) {
					continue;
				}
				Scanner s = new Scanner(robotString).useDelimiter(" ");
				String name = s.next();
				float x = s.nextFloat();
				float y = s.nextFloat();
				float theta = s.nextFloat();
				SimObject robot = new SimObject("splinter", -1, new PointF(x, y));
				robot.setTheta(theta);
				robot.setAttribute("name", name);
				activity.getMapView().addRobot(robot);
				robotNames.add(name);
				subscribeRobot(lcm, name);
			}
		} else if (command.equals("objects")) {
			for (String objString : message.substring(space).split(";")) {
				if (objString.trim().length() == 0) {
					continue;
				}
				Scanner s = new Scanner(objString).useDelimiter(" ");
				String name = s.next();
				int id = s.nextInt();
				float x = s.nextFloat();
				float y = s.nextFloat();
				float theta = s.nextFloat();
				SimObject sim = new SimObject(name, id, new PointF(x, y));
				sim.setTheta(theta);
				activity.getMapView().addObject(sim);
			}
		} else if (command.equals("text")) {
			activity.setPropertiesText(message.substring(space));
		} else if (command.equals("pickup-object")) {
			//TODO
			// Have the robot pickup the object.
			Scanner s = new Scanner(message.substring(space));
			String name = s.next();
			int id = s.nextInt();
			activity.getMapView().pickUpObject(name, id);
		}
		IMapView mv = activity.getMapView();
		if (mv != null) {
			mv.draw();
		}
	}
	

	// Handles a UDP message that was sent from the server.
	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
		if (channel.startsWith("POSE_seek")) {
			try {
				pose_t p = new pose_t(ins);
				PointF robotLocation = new PointF((float) p.pos[0],
						(float) p.pos[1]);

				float theta = (float) (LinAlg.quatToRollPitchYaw(p.orientation)[2] * 180.0f / Math.PI);
				SimObject robot = activity.getMapView().getRobot(
						channel.split("_")[1]);
				if (robot != null) {
					robot.setLocation(robotLocation);
					robot.setTheta(theta);
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else if (channel.startsWith("SIM_LIDAR_FRONT_seek")) {
			try {
				laser_t l = new laser_t(ins);
				SimObject robot = activity.getMapView().getRobot(
						channel.split("_")[3]);
				robot.setLidar(l);
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else if (channel.startsWith("LIDAR_LOWRES_seek")) {
			try {
				laser_t l = new laser_t(ins);
				SimObject robot = activity.getMapView().getRobot(
						channel.split("_")[2]);
				robot.setLowresLidar(l);
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else if (channel.startsWith("WAYPOINTS_seek")) {
			try {
				waypoint_list_t w = new waypoint_list_t(ins);
				SimObject robot = activity.getMapView().getRobot(
						channel.split("_")[1]);
				robot.setWaypoints(w);
			} catch (IOException e) {
				e.printStackTrace();
			}
		} else {
			System.out.println("unknown channel: " + channel);
		}
		IMapView mv = activity.getMapView();
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
			if (lcm == null) {
				lcm = new LCM(lcmConnectionString);
				subscribeAllRobots(lcm, robotNames);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public Thread tcpListener = new Thread() {
		public void run() {
			while (true) {
				try {
					synchronized (RobotSession.this.lock) {
						String message = tcpScanner.next();
						RobotSession.this.handleMessage(message);
					}
				} catch (NoSuchElementException e) {
					e.printStackTrace();
				} catch (Exception e) {
					e.printStackTrace();
				}
			}
		};
	};
	
	private void subscribeAllRobots(LCM lcm, ArrayList<String> robots) {
		for (String robot : robots) {
			subscribeRobot(lcm, robot);
		}
	}
	
	private void subscribeRobot(LCM lcm, String robot) {
		if (lcm == null)
		{
			return;
		}
		lcm.subscribe("POSE_" + robot, this);
		lcm.subscribe("SIM_LIDAR_FRONT_" + robot, this);
		lcm.subscribe("LIDAR_LOWRES_" + robot, this);
		lcm.subscribe("WAYPOINTS_" + robot, this);
	}
	
	public List<String> getRobotNames() {
		return robotNames;
	}
}