package edu.umich.soarrobot.SoarRobotTablet.network;

import android.app.ProgressDialog;
import android.os.Message;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;

public class RobotSession extends Thread {

	SoarRobotTablet activity;
	String server;
	int port;
	
	public RobotSession(SoarRobotTablet activity, String server, int port) {
		this.activity = activity;
		this.server = server;
		this.port = port;
	}
	
	@Override
	public void run() {
		try {
			activity.showAlert("Connecting to server at " + server + ":" + port);
			try {
				sleep(5000);
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
			activity.showAlert("Connected to server at " + server + ":" + port);

		} catch (Exception e) {
			e.printStackTrace();
		}
	}
	
}
