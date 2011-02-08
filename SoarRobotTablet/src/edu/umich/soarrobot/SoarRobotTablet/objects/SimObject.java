/**
 * This class handles the representation of objects in the simulation.
 * Because the definitions for the different types of objects are sent
 * to us over the network when the simulation begins, we can't hard-code
 * those definitions into this class -- we need to be more flexible.
 */

package edu.umich.soarrobot.SoarRobotTablet.objects;

import java.util.HashMap;

import edu.umich.soarrobot.SoarRobotTablet.layout.MapView;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.PointF;
import android.graphics.Paint.Style;

public class SimObject {
	
	/*
	 * Static variables and methods
	 */
	
	// First key is the object class name
	// Second key is the name of the property.
	// Value is of type String or Float, or an array
	// of one of those types.
	private static HashMap<String, HashMap<String, Object>> classes;
	
	// The highest id value that's been assigned so far.
	private static int maxID;
	
	public static void init(HashMap<String, HashMap<String, Object>> classes) {
		SimObject.classes = classes;
		maxID = 0;
	}
	
	/*
	 * Instance variables
	 */
	
	private String type;
	private HashMap<String, Object> attributes;
	private PointF location;
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
		if (attributes.containsKey("size")) {
			float[] sizes = (float[]) attributes.get("size");
			size = new PointF(sizes[0], sizes[1]);
		}
	}
	
	/**
	 * Draws the object to the canvas.
	 * Needs to be implemented.
	 * @param c
	 */
	public void draw(Canvas c, Paint p) {
		c.translate(location.x, location.y);
		p.setStrokeWidth(selected ? 3.0f / MapView.PX_PER_METER : 1.0f / MapView.PX_PER_METER);
		p.setColor(color);
		p.setStyle(Style.FILL);
		c.drawRect(0.0f, 0.0f, size.x, size.y, p);
		p.setColor(selected ? Color.RED : Color.BLACK);
		p.setStyle(Style.STROKE);
		c.drawRect(0.0f, 0.0f, size.x, size.y, p);
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
	
	/*
	 * Accessor methods.
	 */
	
	public String getType() {
		return type;
	}
	
	public boolean isOfType(String type) {
		return this.type.equals(type);
	}
	
	public Object getAttribute(String attribute) {
		return attributes.get(attribute);
	}
	
	public PointF getLocation() {
		return location;
	}
	
	public void setLocation(PointF location) {
		this.location = location;
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
