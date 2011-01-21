package edu.umich.soarrobot.SoarRobotTablet;

import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession;
import android.app.Activity;
import android.app.ProgressDialog;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
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
	
	ProgressDialog pd;
	
	private OnClickListener buttonListener = new OnClickListener() {
		@Override
		public void onClick(View v) {
			propertiesTextView.setText(commandsEditText.getText());
		}
	};

	/** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.main);
        mapView = (MapView) findViewById(R.id.mapView);
        mapView.setActivity(this);
        propertiesTextView = (TextView) findViewById(R.id.propertiesTextView);
        commandsEditText = (EditText) findViewById(R.id.commandsEditText);
        robotSession = new RobotSession(this, "141.212.109.131", 50422);
        robotSession.start();
        ((Button)findViewById(R.id.commandsButton)).setOnClickListener(buttonListener);
    }
    
	public void showAlert(final String message) {
		runOnUiThread(new Runnable() {
			public void run() {
				Toast t = Toast.makeText(SoarRobotTablet.this, message, Toast.LENGTH_LONG);
				t.show();
			}
		});
	}
}