package edu.umich.soarrobot.SoarRobotTablet.objects;

import javax.microedition.khronos.opengles.GL10;

import android.graphics.Rect;

public abstract class SimArea
{   
    protected Rect rect;
    
    protected SimArea(Rect r) {
        this.rect = r;
    }
    
    public abstract void draw(GL10 gl);
    
}
