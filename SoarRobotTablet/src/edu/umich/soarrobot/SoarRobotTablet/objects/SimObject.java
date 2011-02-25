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

/**
 * a=This class handles the representation of objects in the simulation.
 * Because the definitions for the different types of objects are sent
 * to us over the network when the simulation begins, we can't hard-code
 * those definitions into this class -- we need to be more flexible.
 */

package edu.umich.soarrobot.SoarRobotTablet.objects;

import java.util.HashMap;
import java.util.Scanner;

import javax.microedition.khronos.opengles.GL;
import javax.microedition.khronos.opengles.GL10;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Path;
import android.graphics.PointF;
import april.lcmtypes.laser_t;
import april.lcmtypes.waypoint_list_t;
import april.lcmtypes.waypoint_t;
import edu.umich.soarrobot.SoarRobotTablet.layout.GLUtil;
import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;

public class SimObject {
	
	/*
	 * Static variables and methods
	 */
	
	// First key is the object class name
	// Second key is the name of the property.
	// Value is of type String or Float, or an array
	// of one of those types.
	private static HashMap<String, HashMap<String, String>> classes;
	
	// The highest id value that's been assigned so far.
	private static int maxID;
	
	public static void init(String classesString) {
		SimObject.classes = new HashMap<String, HashMap<String,String>>();
		for (String classString : classesString.split(";")) {
			Scanner s = new Scanner(classString);
			s.useDelimiter(" ");
			String name = s.next();
			s.next("\\{");
			HashMap<String, String> properties = new HashMap<String, String>();
			while(true) {
				String key = s.next();
				if (key.equals("}")) {
					break;
				}
				s.next(":");
				String value = s.next();
				s.next(",");
				properties.put(key, value);
			}
			classes.put(name, properties);
		}
		maxID = 0;
	}
	
	public static String[] getClassNames() {
	    return classes.keySet().toArray(new String[] {});
	}
	
	/*
	 * Instance variables
	 */
	
	private String type;
	private HashMap<String, String> attributes;
	private PointF location;
	private float theta;
	private PointF size;
	private int color;
	private int id;
	private boolean selected;
	
	private laser_t lidar;
	private float lidarTheta;
	private PointF lidarLocation;

    private laser_t lowresLidar;
    private float lowresLidarTheta;
    private PointF lowresLidarLocation;
    
    private waypoint_list_t waypoints;
	
	/*
	 * Instance methods
	 */
	
	/**
	 * Class constructor.
	 * 
	 * @param type
	 * @param instanceAttributes
	 */
	public SimObject(String type, PointF location) {
		this.type = type;
		this.attributes = classes.get(type);
		this.location = location;
		this.theta = 0.0f;
		this.id = maxID;
		++maxID;
		selected = false;
		lidar = null;
		lowresLidar = null;
		waypoints = null;
		
		// Set default values
		color = Color.BLACK;
		if (attributes.containsKey("color")) {
			try {
				String colorString = (String) attributes.get("color");
				color = Color.parseColor(colorString);
			} catch (ClassCastException e) {
				e.printStackTrace();
			} catch (IllegalArgumentException e) {
				e.printStackTrace();
			}
		}
		size = new PointF(0.5f, 0.5f);
	}

    /**
     * Draws the object to the canvas.
     * 
     * @param c
     */
    public void draw(GL10 gl)
    {
        drawLidar(lidar, lidarLocation, lidarTheta, gl, Color.RED);
        drawLidar(lowresLidar, lowresLidarLocation, lowresLidarTheta, gl, Color.BLUE);
        drawWaypoints(gl, Color.YELLOW);
        //c.translate(location.x, location.y);
        //c.rotate(-theta);
		//p.setColor(color);
		//p.setStyle(Style.FILL);
		if (type.equals("splinter")) {
			GLUtil.drawCube(gl, location.x - size.x / 2.0f, location.y - size.y / 2.0f, -1.0f, size.x, size.y, 1.0f, Color.RED);
			/*
			Path path = new Path();
			path.lineTo(-1.0f, 0.4f);
			path.lineTo(-1.0f, -0.4f);
			path.lineTo(0.0f, 0.0f);
			c.drawPath(path, p);
			p.setColor(selected ? Color.RED : Color.BLACK);
			p.setStyle(Style.STROKE);
			c.drawPath(path, p);
			*/
		} else {
			GLUtil.drawCube(gl, location.x - size.x / 2.0f, location.y - size.y / 2.0f, -1.0f, size.x, size.y, 1.0f, color);
/*
			c.drawRect(-size.x / 2.0f, -size.y / 2.0f, size.x / 2.0f, size.y / 2.0f, p);
			p.setColor(selected ? Color.RED : Color.BLACK);
			p.setStyle(Style.STROKE);
            c.drawRect(-size.x / 2.0f, -size.y / 2.0f, size.x / 2.0f, size.y / 2.0f, p);
            */
		}
	}
	
	/*
	 * Utility methods
	 */
	
	/**
	 * Check if a point intersects this <code>SimObject</code>.
	 * @param p The point to check.
	 * @return true if the point <code>p</code> intersects this <code>SimObject</code>.
	 */
	public boolean intersectsPoint(PointF p) {
		// Might be different for non-rectangle shapes
		return p.x >= location.x && p.x < location.x + size.x
				&& p.y >= location.y && p.y < location.y + size.y;
	}
	
    public String getPropertiesString() {
        StringBuilder sb = new StringBuilder();
        sb.append("x:" + location.x + "  y:" + location.y + " ");
        for (String str : attributes.keySet()) {
            sb.append(str + ":" + attributes.get(str) + "  ");
        }
        return sb.toString();
          
    }
    
    private static void drawLidar(laser_t lidar, PointF lidarLocation, float lidarTheta, GL10 gl, int color) {
        if (lidar == null) {
            return;
        }
        //c.save();
        gl.glMatrixMode(GL10.GL_MODELVIEW);
        gl.glPushMatrix();
        //c.translate(lidarLocation.x, lidarLocation.y);
        //c.rotate(-lidarTheta);
        gl.glTranslatef(lidarLocation.x, lidarLocation.y, 0.0f);
        gl.glRotatef(lidarTheta, 0.0f, 0.0f, 1.0f);
        //p.setStyle(Style.FILL);
        for (int i = 0; i < lidar.nranges; ++i) {
            float range = lidar.ranges[i];
            float angle = lidar.rad0 + lidar.radstep * i;
            float dx = (float)Math.cos(angle) * range;
            float dy = (float)Math.sin(angle) * range;
            GLUtil.drawCube(gl, -0.1f + dx, -0.1f + dy, -0.5f, 0.2f, 0.2f, 0.2f, color);
            /*
            c.save();
            c.translate(dx, dy);
            c.drawCircle(0.0f, 0.0f, 0.1f, p);
            c.restore();
            */
        }
        gl.glPopMatrix();
        //c.restore();
    }
    
    private void drawWaypoints(GL10 gl, int color) {
        if (waypoints == null) {
            return;
        }
        for (waypoint_t w : waypoints.waypoints) {
        	GLUtil.drawCube(gl, (float) w.xLocal - 0.1f, (float) w.yLocal - 0.1f, -0.5f, 0.2f, 0.2f, 0.2f, color);
        	/*
            c.save();
            c.translate((float)w.xLocal, (float)-w.yLocal);
            c.drawCircle(0.0f, 0.0f, 0.1f, p);
            c.restore();
            */
        }
    }
	
	/*
	 * Accessor methods.
	 */
	
	public String getType() {
		return type;
	}
	
	public boolean isOfType(String type) {
		return this.type.equals(type);
	}
	
	public String getAttribute(String attribute) {
		return attributes.get(attribute);
	}
	
	
	
	public void setAttribute(String attribute, String value) {
		attributes.put(attribute, value);
	}
	
	public PointF getLocation() {
		return location;
	}
	
	public void setLocation(PointF location) {
		this.location = location;
	}
	
	public float getTheta()
	{
		return theta;
	}
	
	public void setTheta(float theta) {
		this.theta = theta;
	}
	
	public void setSelected(boolean selected) {
		this.selected = selected;
	}
	
	public PointF getSize() {
		return size;
	}
	
	public int getID() {
		return id;
	}

    public void setLidar(laser_t lidar)
    {
       this.lidar = lidar;
       lidarTheta = theta;
       lidarLocation = location;
    }
    
    public void setLowresLidar(laser_t lidar)
    {
       this.lowresLidar = lidar;
       lowresLidarTheta = theta;
       lowresLidarLocation = location;
    }

    public void setWaypoints(waypoint_list_t w)
    {
        waypoints = w;
    }
}
