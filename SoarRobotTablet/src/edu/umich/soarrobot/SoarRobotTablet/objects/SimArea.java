package edu.umich.soarrobot.SoarRobotTablet.objects;

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
    
    public SimArea(int id, Rect r, String type) {
    	super("area", id, new PointF(r.left, r.bottom));
    	this.size = new PointF(r.width(), -r.height());
        this.rect = r;
        lightsOn = true;
        doorClosed = false;
    }
    
    public void draw(GL10 gl) {
    	int color = lightsOn ? Color.WHITE : Color.GRAY;
    	if (doorClosed) {
    		GLUtil.drawCube(gl, rect.left, rect.bottom, 0.0f, rect.right - rect.left,
    				rect.top - rect.bottom, 1.0f, color);
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
    	this.doorClosed = doorClosed;
    }
    
    public int getID() {
    	return id;
    }
    
    public Rect getRect() {
    	return rect;
    }
}
