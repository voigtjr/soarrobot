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
package edu.umich.soarrobot.SoarRobotTablet;

import java.util.ArrayList;
import java.util.List;

import com.sun.org.apache.bcel.internal.generic.NEW;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;
import edu.umich.soarrobot.SoarRobotTablet.layout.GLMapView;
import edu.umich.soarrobot.SoarRobotTablet.layout.IMapView;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession.TextMessageListener;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class SoarRobotTablet extends Activity implements TextMessageListener
{

    private GLMapView mapView;

    private RobotSession robotSession;

    public TextView propertiesTextView;

    private EditText commandsEditText;

    private SimObject selectedObject;

    ProgressDialog pd;
    
    private ArrayList<String> messageHistory = new ArrayList<String>();
    
    private static SoarRobotTablet instance;
    
    public static SoarRobotTablet getInstance() {
    	return instance;
    }
    
    private OnClickListener toggleFollowListener = new OnClickListener() {
		@Override
		public void onClick(View v) {
			mapView.toggleFollow();
		}
	};

    private OnClickListener buttonListener = new OnClickListener()
    {
        @Override
        public void onClick(View v)
        {
            // send the message
        	List<String> robotNames = robotSession.getRobotNames();
        	SimObject robot = mapView.getFollow();
        	if (robot == null) {
                showAlert("No robot selected.", Toast.LENGTH_SHORT);
        		return;
        	}
        	if(robotNames.size() > 0) {
                String message = commandsEditText.getText().toString();
                String robotName = robot.getAttribute("name");
                robotSession.sendMessage("text tablet " + robotName + " " + message);
        	}
            commandsEditText.setText("");
        }
    };

    private OnClickListener pauseListener = new OnClickListener()
    {
        @Override
        public void onClick(View v)
        {
            robotSession.sendMessage("pause");
        }
    };
    
    private OnClickListener addObject = new OnClickListener()
    {
        
        @Override
        public void onClick(View v)
        {
            final CharSequence[] items = SimObject.getClassNames();
            //final CharSequence[] items = {"one", "two"};
            AlertDialog.Builder builder = new AlertDialog.Builder(SoarRobotTablet.this)
                    .setTitle("Add Object")
                    .setItems(items, new DialogInterface.OnClickListener()
                    {
                        @Override
                        public void onClick(DialogInterface dialog, int which)
                        {
                            SoarRobotTablet.this.mapView
                                    .setNextClass(items[which]);
                        }
                    });
            builder.show();
        }
    };

    private OnClickListener zoomIn = new OnClickListener()
    {

        @Override
        public void onClick(View v)
        {
            mapView.zoomIn();
        }
    };

    private OnClickListener zoomOut = new OnClickListener()
    {

        @Override
        public void onClick(View v)
        {
            mapView.zoomOut();
        }
    };

    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState)
    {
        try
        {
            super.onCreate(savedInstanceState);
            instance = this;
            setContentView(R.layout.main);
            mapView = (GLMapView) findViewById(R.id.mapView);
            mapView.setActivity(this);
            propertiesTextView = (TextView) findViewById(R.id.propertiesTextView);
            commandsEditText = (EditText) findViewById(R.id.commandsEditText);
            setSelectedObject(null);
            try
            {
                //robotSession = new RobotSession(this, "141.212.109.139", 12122); // Miller's ip
                robotSession = new RobotSession(this, "141.212.109.194", 12122); // Kevin's ip
                robotSession.addTextMessageListener(this);
                robotSession.start();
            }
            catch (Exception e)
            {
                e.printStackTrace();
                showAlert("Failed to connect to server", Toast.LENGTH_LONG);
            }
            ((Button) findViewById(R.id.commandsButton))
                    .setOnClickListener(buttonListener);
            ((Button) findViewById(R.id.pauseButton))
                    .setOnClickListener(pauseListener);
            ((Button) findViewById(R.id.zoomIn)).setOnClickListener(zoomIn);
            ((Button) findViewById(R.id.zoomOut)).setOnClickListener(zoomOut);
            ((Button) findViewById(R.id.addObject)).setOnClickListener(addObject);
            ((Button) findViewById(R.id.toggleFollow)).setOnClickListener(toggleFollowListener);
            
            
        }
        catch (RuntimeException e)
        {
            e.printStackTrace();
        }
    }

    public void showAlert(final String message, final int length)
    {
        runOnUiThread(new Runnable()
        {
            public void run()
            {
                Toast t = Toast.makeText(SoarRobotTablet.this, message, length);
                t.show();
            }
        });
    }

    /*
     * Accessor methods
     */

    public IMapView getMapView()
    {
        return mapView;
    }

    public RobotSession getRobotSession()
    {
        return robotSession;
    }

    public void setSelectedObject(SimObject obj)
    {
        if (selectedObject != null)
        {
            selectedObject.setSelected(false);
        }
        selectedObject = obj;
        if (obj != null)
        {
            obj.setSelected(true);
            propertiesTextView
                    .setText("" + obj.getID() + ": " + obj.getPropertiesString());
        }
        else
        {
            propertiesTextView.setText("NULL");
        }
    }

    // Lifecycle

    @Override
    protected void onPause()
    {
        super.onPause();

    	/*   if (robotSession != null)
        {
            robotSession.pause();
        }
        */
    }

    @Override
    protected void onResume()
    {
        super.onResume();
        if (robotSession != null)
        {
            robotSession.unPause();
        }
    }

    public void setPropertiesText(final String text)
    {
        runOnUiThread(new Runnable()
        {
            public void run()
            {
                propertiesTextView.setText(text);
            }
        });
    }
    
    public void textMessageReceived(final String text)
    {
    	messageHistory.add(text);
    	setPropertiesText(text);
    }
    
    public ArrayList<String> getTextMessageHistory()
    {
    	return messageHistory;
    }
    
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
    	getMenuInflater().inflate(R.menu.main_menu, menu);
    	return true;
    }
    
    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle item selection
        switch (item.getItemId()) {
        case R.id.showChatMenuItem:
        	Intent i = new Intent(this, ChatActivity.class);
        	startActivity(i);
        	break;
        default:
        	break;
        }
    	return true;
    }
}
