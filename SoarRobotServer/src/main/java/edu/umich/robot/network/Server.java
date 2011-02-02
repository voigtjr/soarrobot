package edu.umich.robot.network;

import java.io.IOException;
import java.io.PrintWriter;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.NoSuchElementException;
import java.util.Scanner;

import javax.swing.JOptionPane;

import edu.umich.robot.Controller;
import edu.umich.robot.TabletLCM;

/**
 * Listens for commands over the network.
 * Used to talk to the tablet.
 * @author miller
 *
 */
public class Server {
	
	ServerSocket socket;
	boolean running;
	Controller controller;
	
	TabletLCM lcm;
	
	public Server(int port) throws IOException {
		socket = new ServerSocket(port);
		running = true;
	}
	
	public void start() {
		running = true;
		new Thread() {
			public void run() {
				while (running) {
					try {
						Socket client = socket.accept();
						startLCM(client.getInetAddress(), client.getPort());
						Scanner scanner = new Scanner(client.getInputStream()).useDelimiter("\n");
						boolean scanning = true;
						try {
							while (scanning) {
								// Read a command from the client and respond
								// Print something to the command line for debugging
								String line = scanner.next();
								System.out.println("Got command from client " + client.getInetAddress() + ":" + client.getPort() + ":\n" + line);
								String response = handleCommand(line);
								PrintWriter out = new PrintWriter(client.getOutputStream());
								out.println(response);
								out.flush();
								if (line.trim().equalsIgnoreCase("quit")) {
									scanning = false;
								}
							}
						} catch (IOException e) {
							e.printStackTrace();
						} catch (NoSuchElementException e) {
							e.printStackTrace();
						} finally {
							client.close();
						}
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
	
	private String handleCommand(String command) {
		if (controller == null) {
			return "No controller found";
		}
		if (command.equalsIgnoreCase("pause")) {
			return controller.toggleSoarRunState() ? "Soar started" : "Soar paused";
		}
		return "Invalid command: " + command;
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
	
}
