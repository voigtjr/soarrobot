package edu.umich.robot.network;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Scanner;

import com.google.common.collect.ImmutableList;

import edu.umich.robot.Controller;
import edu.umich.robot.TabletLCM;
import edu.umich.robot.events.ObjectAddedEvent;
import edu.umich.robot.metamap.AbridgedAreaDescription;
import edu.umich.robot.metamap.AbridgedGateway;
import edu.umich.robot.metamap.AreaDescription;
import edu.umich.robot.metamap.Gateway;
import edu.umich.robot.metamap.SquareArea;
import edu.umich.robot.metamap.VirtualObject;
import edu.umich.robot.metamap.VirtualObjectTemplate;
import edu.umich.robot.splinter.Splinter;
import edu.umich.robot.util.Pose;
import edu.umich.robot.util.events.RobotEvent;
import edu.umich.robot.util.events.RobotEventListener;

/**
 * Listens for commands over the network.
 * Used to talk to the tablet.
 * @author miller
 *
 */
public class Server implements RobotEventListener {
	
	ServerSocket socket;
	boolean running;
	Controller controller;
	PrintWriter out;
	
	TabletLCM lcm;
	
	public Server(int port) throws IOException {
		socket = new ServerSocket(port);
		running = true;
		out = null;
	}
	
	public void start() {
		running = true;
		new Thread() {
			public void run() {
				while (running) {
					try {
						Socket client = socket.accept();
						startLCM(client.getInetAddress(), client.getPort());
						//startLCM(InetAddress.getByName("10.0.2.15"), client.getPort());
						Scanner scanner = new Scanner(client.getInputStream()).useDelimiter("\n");
						boolean scanning = true;
                        out = new PrintWriter(client.getOutputStream());
						try {
							while (scanning) {
								// Read a command from the client and respond
								// Print something to the command line for debugging
								String line = scanner.next();
								System.out.println("Got command from client " + client.getInetAddress() + ":" + client.getPort() + ":\n" + line);
								String response = handleCommand(line);
								System.out.println("Returning to client:\n" + response);
								sendMessage(response);
								if (line.trim().equalsIgnoreCase("quit")) {
									scanning = false;
								}
							}
						} catch (NoSuchElementException e) {
							e.printStackTrace();
						} finally {
							client.close();
						}
						out = null;
					} catch (IOException e) {
						e.printStackTrace();
					}
					stopLCM();
				}
				try {
					socket.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
				System.out.println("Server done");
			}
		}.start();
	}
	
	private synchronized void sendMessage(String message) {
	    if (out != null && message != null) {
	        out.println(message);
            out.flush();
	    }
    }
	
	private String handleCommand(String command) {
		if (controller == null) {
			return "No controller found";
		}
		
		// map
		// Return map description
		if (command.equalsIgnoreCase("map")) {
			StringBuilder sb = new StringBuilder();
			for (AreaDescription ad : controller.getAreaList()) {
				if (ad instanceof SquareArea) {
					AbridgedAreaDescription aad = abridgeAreaDescription((SquareArea)ad);
					sb.append(aad.toString());
					sb.append(" ; ");
				}
			}
			return "map " + sb.toString();
		}
		
		// classes
		// Return description of object classes
		if (command.equalsIgnoreCase("classes")) {
			StringBuilder sb = new StringBuilder();
			for (VirtualObjectTemplate vot : controller.getTemplates()) {
				sb.append(vot.getName());
				sb.append(" { ");
				Map<String, String> properties = vot.getProperties();
				for (String key : properties.keySet()) {
					sb.append(key);
					sb.append(" : ");
					sb.append(properties.get(key));
					sb.append(" , ");
				}
				sb.append("};");
			}
			return "classes " + sb.toString();
		}
		
		// objects
		// Return objects description
		if (command.equalsIgnoreCase("objects")) {
			StringBuilder sb = new StringBuilder();
			for (VirtualObject obj : controller.getPlacedObjects()) {
				sb.append(stringForVirtualObject(obj));
			}
			return "objects " + sb.toString();
		}
		
		// robots
		// Return robots description
		if (command.equalsIgnoreCase("robots")) {
			StringBuilder sb = new StringBuilder();
			for (Object obj : controller.getAllRobots()) {
				if (obj instanceof Splinter) {
					Splinter s = (Splinter)obj;
					sb.append(s.getName());
					sb.append(' ');
					Pose p = s.getOutput().getPose();
					sb.append(p.getX());
					sb.append(' ');
					sb.append(p.getY());
					sb.append(' ');
					sb.append(p.getYaw());
					sb.append(';');
				}
			}
			return "robots " + sb.toString();
		}
		
		// pause
		// Toggle Soar's run state
		if (command.equalsIgnoreCase("pause")) {
			return "text " + (controller.toggleSoarRunState() ? "Soar started" : "Soar paused");
		}
		
		String[] tokens = command.split(" ");
		if (tokens[0].equalsIgnoreCase("object") && tokens.length >= 4) {
		    controller.addObject(tokens[1], new double[] {Double.parseDouble(tokens[2]), -Double.parseDouble(tokens[3])});
		    return "Created " + tokens[1] + " at (" + tokens[2] + ", " + tokens[3] + ")";
		}
		
		return "text Invalid command: " + command;
	}
	
	private static String stringForVirtualObject(VirtualObject obj) {
        Pose p = obj.getPose();
	    return obj.getName() + " " + p.getX() + " " + p.getY() + " " + p.getYaw() + ";";
	}
	
	public static AbridgedAreaDescription abridgeAreaDescription(SquareArea sa) {
		int[] pr = sa.getPixelRect();
		ImmutableList<Double> xywh = new ImmutableList.Builder<Double>().add((double)pr[0], (double)pr[1], (double)pr[2], (double)pr[3]).build();
		ImmutableList.Builder<AbridgedGateway> gatewaysBuilder = new ImmutableList.Builder<AbridgedGateway>();
		for (Gateway g : sa.getGateways()) {
			gatewaysBuilder.add(abridgeGateway(g));
		}
		return new AbridgedAreaDescription(sa.getId(), xywh, gatewaysBuilder.build());
	}
	
	public static AbridgedGateway abridgeGateway(Gateway g) {
		ImmutableList<Double> xy = new ImmutableList.Builder<Double>().add(g.getPose().getX(), g.getPose().getY()).build();
		return new AbridgedGateway(g.getId(), xy);
	}
	
	public void stop() {
		running = false;
	}
	
	/**
	 * Tests functionality.
	 * @param args
	 */
	public static void main(String[] args) {
		Socket client = null;
		try {
			new Server(12122).start();
		} catch (IOException e) {
			e.printStackTrace();
		} finally {
			if (client != null) {
				try {
					client.close();
				} catch (IOException e) {
					e.printStackTrace();
				}
			}
		}
	}

	public void setController(Controller controller) {
		this.controller = controller;
	}
	

    private synchronized void startLCM(InetAddress client, int port)
    {
        if (lcm == null)
        {
			String connectionString = "udp://" + client.getHostAddress() + ":" + port;
			try {
				lcm = new TabletLCM(connectionString);
				System.out.println("Started UDP LCM forwarding to client: " + connectionString);
			} catch (IllegalArgumentException e) {
				e.printStackTrace();
			} catch (IOException e) {
				e.printStackTrace();
			}
        }
    }
    
    private synchronized void stopLCM() {
        if (lcm != null)
        {
            lcm.close();
            lcm = null;
        }
    }

    @Override
    public void onEvent(RobotEvent event)
    {
        if (event instanceof ObjectAddedEvent) {
            ObjectAddedEvent oae = (ObjectAddedEvent) event;
            VirtualObject vo = oae.getObject();
            sendMessage("objects " + stringForVirtualObject(vo));
        }
    }
	
}
