package edu.umich.soarrobot.SoarRobotTablet;

import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class SoarRobotTablet extends Activity {
	
	private MapView mapView;
	private RobotSession robotSession;
	private TextView propertiesTextView;
	private EditText commandsEditText;
	private SimObject selectedObject;
		
	ProgressDialog pd;
	
	private OnClickListener buttonListener = new OnClickListener() {
		@Override
		public void onClick(View v) {
			// send the message
			String message = commandsEditText.getText().toString();
			String response = robotSession.sendMessage(message);
			propertiesTextView.setText(response);
			commandsEditText.setText("");
		}
	};
	
	private OnClickListener pauseListener = new OnClickListener() {
		@Override
		public void onClick(View v) {
			String response = robotSession.sendMessage("pause");
			propertiesTextView.setText(response);
		}
	};
	
	private OnClickListener zoomIn = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			mapView.zoomIn();
		}
	};
	
	private OnClickListener zoomOut = new OnClickListener() {
		
		@Override
		public void onClick(View v) {
			mapView.zoomOut();
		}
	};

	/** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
    	try {
    		super.onCreate(savedInstanceState);
    		setContentView(R.layout.main);
    		mapView = (MapView) findViewById(R.id.mapView);
    		mapView.setActivity(this);
    		propertiesTextView = (TextView) findViewById(R.id.propertiesTextView);
    		commandsEditText = (EditText) findViewById(R.id.commandsEditText);
    		setSelectedObject(null);
    		try {
    			robotSession = new RobotSession(this, "141.212.109.131", 12122);
    			robotSession.start();
    		} catch (Exception e) {
    			e.printStackTrace();
    			showAlert("Failed to connect to server", Toast.LENGTH_LONG);
    		}
    		((Button)findViewById(R.id.commandsButton)).setOnClickListener(buttonListener);
    		((Button)findViewById(R.id.pauseButton)).setOnClickListener(pauseListener);
    		((Button)findViewById(R.id.zoomIn)).setOnClickListener(zoomIn);
    		((Button)findViewById(R.id.zoomOut)).setOnClickListener(zoomOut);
    	} catch (RuntimeException e) {
    		e.printStackTrace();
    	}
    }
    
	public void showAlert(final String message, final int length) {
		runOnUiThread(new Runnable() {
			public void run() {
				Toast t = Toast.makeText(SoarRobotTablet.this, message, length);
				t.show();
			}
		});
	}
	
	/*
	 * Accessor methods
	 */
	
	public MapView getMapView() {
		return mapView;
	}
	
	public RobotSession getRobotSession() {
		return robotSession;
	}
	
	public void setSelectedObject(SimObject obj) {
		if (selectedObject != null) {
			selectedObject.setSelected(false);
		}
		selectedObject = obj;
		if (obj != null) {
			obj.setSelected(true);
			propertiesTextView.setText("" + obj.getID() + ": " + obj.toString());
		} else {
			propertiesTextView.setText("NULL");
		}
	}
	
	// Lifecycle
	
	@Override
	protected void onPause() {
		super.onPause();
		if (robotSession != null) {
			robotSession.pause();
		}
	}
	
	@Override
	protected void onResume() {
		super.onResume();
		if (robotSession != null) {
			robotSession.unPause();
		}
	}
	
	@Override
	protected void onStop() {
		super.onStop();
		if (robotSession != null) {
			robotSession.pause();
		}
		System.exit(0);
	}
}