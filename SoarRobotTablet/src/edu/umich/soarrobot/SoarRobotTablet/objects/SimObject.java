/**
 * This class handles the representation of objects in the simulation.
 * Because the definitions for the different types of objects are sent
 * to us over the network when the simulation begins, we can't hard-code
 * those definitions into this class -- we need to be more flexible.
 */

package edu.umich.soarrobot.SoarRobotTablet.objects;

import java.util.HashMap;
import java.util.Scanner;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Path;
import android.graphics.PointF;
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
		
		// Set default values
		color = Color.BLACK;
		size = new PointF(1.0f, 1.0f);
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
		size = new PointF(1.0f, 1.0f);
	}
	
	/**
	 * Draws the object to the canvas.
	 * Needs to be implemented.
	 * @param c
	 */
	public void draw(Canvas c, Paint p) {
		c.translate(location.x, location.y);
		c.rotate(-theta);
		p.setStrokeWidth(selected ? 3.0f / MapView.PX_PER_METER : 1.0f / MapView.PX_PER_METER);
		p.setColor(color);
		p.setStyle(Style.FILL);
		if (type.equals("splinter")) {
			Path path = new Path();
			path.lineTo(-1.0f, 0.4f);
			path.lineTo(-1.0f, -0.4f);
			path.lineTo(0.0f, 0.0f);
			c.drawPath(path, p);
			p.setColor(selected ? Color.RED : Color.BLACK);
			p.setStyle(Style.STROKE);
			c.drawPath(path, p);
		} else {
			c.drawRect(0.0f, 0.0f, size.x, size.y, p);
			p.setColor(selected ? Color.RED : Color.BLACK);
			p.setStyle(Style.STROKE);
			c.drawRect(0.0f, 0.0f, size.x, size.y, p);
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
}
