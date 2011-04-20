package edu.umich.soarrobot.SoarRobotTablet.objects;

import java.util.ArrayList;
import java.util.List;

import javax.microedition.khronos.opengles.GL10;

import edu.umich.robot.metamap.AbridgedWall;
import edu.umich.soarrobot.SoarRobotTablet.layout.GLUtil;

import android.graphics.Color;
import android.graphics.PointF;
import android.graphics.Rect;

public class SimArea extends SimObject
{   
	private int id;
    private Rect rect;
    private boolean lightsOn;
    private boolean doorClosed;
    private boolean isDoorway;
    private float position;
    
    public SimArea(int id, Rect r, String type) {
    	super("area", id, new PointF(r.left, r.bottom));
    	this.id = id;
    	this.size = new PointF(r.width(), -r.height());
        this.rect = r;
        this.isDoorway = type.equals("door");
        this.position = 0.0f;
        lightsOn = true;
        doorClosed = false;
    }
    
    public void draw(GL10 gl) {
    	int color = (position < 0.5f && isDoorway) ? Color.DKGRAY : (lightsOn ? Color.WHITE : Color.DKGRAY);
    	if (doorClosed) {
    	    position -= 0.04f;
    	    if (position <= -0.5f) {
    	        position = -0.5f;
    	    }
    	} else {
    	    position += 0.04f;
    	    if (position >= 0.5f) {
    	        position = 0.5f;
    	    }
    	}
    	/*
    	 * -0.5f - door is closed
    	 * 0.5f - door is open
    	 * */
    	
    	if (position < 0.5f && isDoorway) {
    		float width = rect.right - rect.left;
    		float height = rect.top - rect.bottom;
    		float centerX = rect.left + width / 2.0f;
    		float centerY = rect.bottom + height / 2.0f;
    		GLUtil.drawCube(gl, centerX, centerY, position, width,
    				height, 1.0f, color);
    	} else {
    		GLUtil.drawRect(gl, rect.left, rect.bottom, 0.0f, rect.right - rect.left,
    				rect.top - rect.bottom, color);
    	}
    }
    
    public boolean getLightsOn() {
    	return lightsOn;
    }
    
    public void setLightsOn(boolean lightsOn) {
    	this.lightsOn = lightsOn;
    }
    
    public boolean getDoorClosed() {
    	return doorClosed;
    }
    
    public void setDoorClosed(boolean doorClosed) {
    	if (!isDoorway) return;
    	this.doorClosed = doorClosed;
    }
    
    public int getID() {
    	return id;
    }
    
    public Rect getRect() {
    	return rect;
    }
    
    public String getPropertiesString() {
        StringBuilder sb = new StringBuilder();
        sb.append(type + " id=" + getID() + "; x=" + location.x + "; y=" + location.y + "; ");
        sb.append("lights=" + (lightsOn ? "on" : "off") + "; ");
        if (isDoorway)
        {
        	sb.append("door-closed=" + doorClosed + "; ");
        }
        for (String str : attributes.keySet()) {
            sb.append(str + "=" + attributes.get(str) + "; ");
        }
        return sb.toString();
    }
    
    @Override
    public ArrayList<TemplateAction> getActions() {
    	ArrayList<TemplateAction> ret = super.getActions();
    	if (isDoorway)
    	{
    		ArrayList<TemplateAction> all = new ArrayList<TemplateAction>();
    		all.addAll(ret);
    		all.add(new TemplateAction("area", "open-door", true, true));
    		all.add(new TemplateAction("area", "close-door", true, true));
    		return all;
    	}
    	return ret;
    }
}
