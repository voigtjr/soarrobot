package edu.umich.soarrobot.SoarRobotTablet.objects;

import javax.microedition.khronos.opengles.GL10;

import android.graphics.Color;
import android.graphics.Point;
import edu.umich.soarrobot.SoarRobotTablet.layout.GLUtil;

public class SimWallTop {
    Point start;
    Point end;
    
    public SimWallTop(Point start, Point end) {
        this.start = start;
        this.end = end;
    }
    
    public void draw(GL10 gl, boolean drawWalls) {
        GLUtil.drawWallTop(gl, start, end, Color.GRAY, drawWalls);
    }
}
