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

import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;
import edu.umich.soarrobot.SoarRobotTablet.network.RobotSession;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.DialogInterface;
import android.os.Bundle;
import android.view.View;
import android.view.View.OnClickListener;
import android.widget.Button;
import android.widget.EditText;
import android.widget.TextView;
import android.widget.Toast;

public class SoarRobotTablet extends Activity
{

    private MapView mapView;

    private RobotSession robotSession;

    public TextView propertiesTextView;

    private EditText commandsEditText;

    private SimObject selectedObject;

    ProgressDialog pd;

    private OnClickListener buttonListener = new OnClickListener()
    {
        @Override
        public void onClick(View v)
        {
            // send the message
            String message = commandsEditText.getText().toString();
            robotSession.sendMessage("text " + message);
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
            setContentView(R.layout.main);
            mapView = (MapView) findViewById(R.id.mapView);
            mapView.setActivity(this);
            propertiesTextView = (TextView) findViewById(R.id.propertiesTextView);
            commandsEditText = (EditText) findViewById(R.id.commandsEditText);
            setSelectedObject(null);
            try
            {
                robotSession = new RobotSession(this, "141.212.109.194", 12122);
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

    public MapView getMapView()
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
        if (robotSession != null)
        {
            robotSession.pause();
        }
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

    @Override
    protected void onStop()
    {
        super.onStop();
        if (robotSession != null)
        {
            robotSession.pause();
        }
        System.exit(0);
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
}