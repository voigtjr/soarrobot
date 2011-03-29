package edu.umich.soarrobot.SoarRobotTablet.objects;


import javax.microedition.khronos.opengles.GL10;

import edu.umich.soarrobot.SoarRobotTablet.layout.GLUtil;

import android.graphics.Color;
import android.graphics.Point;

public class SimWall {
	Point start;
	Point end;
	public SimWall(Point start, Point end)
	{
		this.start = start;
		this.end = end;
	}
	
	public void draw(GL10 gl)
	{
		GLUtil.drawWall(gl, start, end, 1.0f, Color.GRAY);
	}
}
