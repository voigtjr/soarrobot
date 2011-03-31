package edu.umich.soarrobot.SoarRobotTablet;

import java.util.ArrayList;

import android.app.Activity;
import android.os.Bundle;
import android.widget.TextView;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession.TextMessageListener;

public class ChatActivity extends Activity implements TextMessageListener {

	TextView tv;
	
	/** Called when the activity is first created. */
	@Override
	public void onCreate(Bundle savedInstanceState) {
	    super.onCreate(savedInstanceState);
	    setContentView(R.layout.chat);
	    tv = (TextView)findViewById(R.id.chatLogText);
	    tv.setText("Hello, chat");
	    SoarRobotTablet root = SoarRobotTablet.getInstance();
	    RobotSession session = root.getRobotSession();
	    session.addTextMessageListener(this);
	    ArrayList<String> messageHistory = root.getTextMessageHistory();
	    for (String message : messageHistory)
	    {
	    	tv.append("\n" + message);
	    }
	}

	@Override
	public void textMessageReceived(String message) {
		tv.append("\n" + message);
	}

}
