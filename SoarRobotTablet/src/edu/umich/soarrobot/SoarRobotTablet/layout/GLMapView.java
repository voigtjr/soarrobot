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
package edu.umich.soarrobot.SoarRobotTablet.layout;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

import android.content.Context;
import android.graphics.Color;
import android.graphics.PointF;
import android.graphics.Rect;
import android.opengl.GLSurfaceView;
import android.opengl.GLSurfaceView.Renderer;
import android.opengl.GLU;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import edu.umich.robot.metamap.AbridgedAreaDescription;
import edu.umich.robot.metamap.AbridgedAreaDescriptions;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class GLMapView extends GLSurfaceView implements Callback, Renderer, IMapView
{

    public static final String TAG = "MAP_VIEW";
    public static final int PX_PER_METER = 32;
    private static final float zoomRate = 1.5f;
    private SoarRobotTablet activity;
    private PointF camera;
    private float zoom;
    private PointF lastTouch;
    private HashMap<Integer, SimObject> objects;
    private HashMap<String, SimObject> robots;
    private ArrayList<Rect> areas;
    private String nextObjectClass;
        
    public GLMapView(Context context)
    {
        super(context);
        init();
    }

    public GLMapView(Context context, AttributeSet attrs)
    {
        super(context, attrs);
        init();
    }

    private void init()
    {
        SurfaceHolder sh = getHolder();
        sh.addCallback(this);
        camera = new PointF(0.0f, 0.0f);
        zoom = -70.0f;
        lastTouch = new PointF(-1.0f, -1.0f);
        objects = new HashMap<Integer, SimObject>();
        robots = new HashMap<String, SimObject>();
        areas = new ArrayList<Rect>();
        nextObjectClass = null;
        
        setRenderer(this);
        //setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
    }

    public void setActivity(SoarRobotTablet activity)
    {
        this.activity = activity;
    }

    public void addObject(SimObject object)
    {
        objects.put(object.getID(), object);
    }

    public void removeObject(SimObject object)
    {
        objects.remove(object.getID());
    }

    public SimObject getObject(int objectID)
    {
        return objects.get(objectID);
    }

    public void addRobot(SimObject robot)
    {
        robots.put(robot.getAttribute("name"), robot);
    }

    public void removeRobot(String name)
    {
        robots.remove(name);
    }

    public void removeRobot(SimObject robot)
    {
        robots.remove(robot.getAttribute("name"));
    }

    public SimObject getRobot(String name)
    {
        return robots.get(name);
    }

    public void draw()
    {
        requestRender();
    }

    @Override
    public boolean onTouchEvent(MotionEvent event)
    {
        int action = event.getAction();
        if (action == MotionEvent.ACTION_MOVE)
        {
            PointF touch = new PointF(event.getX(), event.getY());
            if (lastTouch.x >= 0.0f && lastTouch.y >= 0.0f)
            {
                synchronized (camera)
                {
                    camera.x -= (touch.x - lastTouch.x) / PX_PER_METER;
                    camera.y += (touch.y - lastTouch.y) / PX_PER_METER;
                }
                draw();
            }
            lastTouch = touch;
        }
        else if (action == MotionEvent.ACTION_DOWN)
        {
            PointF touch = new PointF(event.getX(), event.getY());
            // Transform the touch into meter coordinates
            touch.x /= PX_PER_METER;
            touch.y /= PX_PER_METER;
            // Translate the touch according to the camera
            touch.x += camera.x;
            touch.y += camera.y;
            // Taking the zoom factor into account
            touch.x /= zoom;
            touch.y /= zoom;
            if (nextObjectClass != null)
            {
                activity.getRobotSession().sendMessage("object " + nextObjectClass + " " + touch.x + " " + touch.y);
                nextObjectClass = null;
            }
            else
            {
                try
                {
                    boolean selected = false;
                    synchronized (objects)
                    {
                        for (SimObject obj : robots.values())
                        {
                            if (obj.intersectsPoint(touch))
                            {
                                activity.setSelectedObject(obj);
                                selected = true;
                                break;
                            }
                        }
                        if (!selected)
                        {
                            for (SimObject obj : objects.values())
                            {
                                if (obj.intersectsPoint(touch))
                                {
                                    activity.setSelectedObject(obj);
                                    selected = true;
                                    break;
                                }
                            }
                        }
                    }
                    draw();
                }
                catch (NullPointerException e)
                { // Don't know why this is happening
                    e.printStackTrace();
                }
            }
            lastTouch.x = (int) event.getX();
            lastTouch.y = (int) event.getY();
        }
        else if (action == MotionEvent.ACTION_CANCEL
                || action == MotionEvent.ACTION_UP)
        {
            lastTouch.x = -1.0f;
            lastTouch.y = -1.0f;
        }
        return true;
    }

    public void deserializeMap(String map)
    {
        String[] areaList = map.split(";");
        for (String area : areaList)
        {
            if (area.trim().length() == 0)
            {
                continue;
            }
            System.out.println("Parsing area:\n" + area);
            AbridgedAreaDescription aad = AbridgedAreaDescriptions
                    .parseArea(area);
            List<Double> xywh = aad.xywh;
            int x = (int) (double) xywh.get(0);
            int y = (int) (double) xywh.get(1);
            int w = (int) (double) xywh.get(2);
            int h = (int) (double) xywh.get(3);
            int left, right, top, bottom;
            if (w >= 0) {
            	left = x;
            	right = x + w;
            } else {
            	left = x + w;
            	right = x;
            }
            if (h >= 0) {
            	bottom = y;
            	top = y + h;
            } else {
            	bottom = y;
            	top = y - h;
            }
            synchronized (areas) {
                areas.add(new Rect(left, top, right, bottom));	
			}
        }
    }

    public void zoomIn()
    {
        zoom += 1;
        draw();
    }

    public void zoomOut()
    {
        zoom -= 1;
        draw();
    }

    public void setNextClass(CharSequence nextClass)
    {
        nextObjectClass = nextClass.toString();
    }

	@Override
	public void onDrawFrame(GL10 gl) {
		/*
		 * Paint p = new Paint();

            p.setColor(Color.WHITE);
            for (Rect r : areas)
            {
                c.save();
                c.translate(r.left, r.top);
                c.drawRect(0, 0, r.width(), r.height(), p);
                c.restore();
            }

            for (SimObject object : objects.values())
            {
                c.save();
                object.draw(c, p);
                c.restore();
            }

            for (SimObject robot : robots.values())
            {
                c.save();
                robot.draw(c, p);
                c.restore();
            }

            c.restore();
            
		 */
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
        gl.glMatrixMode(GL10.GL_MODELVIEW);
		gl.glLoadIdentity();
        GLU.gluLookAt(gl, camera.x, camera.y, zoom, camera.x, camera.y, 0.0f, 0f, 1.0f, 0.0f);
        
		synchronized (areas) {
			for (Rect r : areas) {
				GLUtil.drawRect(gl, r.left, r.bottom, 0.0f, r.right - r.left, r.top - r.bottom, Color.WHITE);
			}
		}
		
		synchronized (objects) {
			for (SimObject obj : objects.values()) {
				obj.draw(gl);
			}
		}
		
		synchronized (robots) {
			for (SimObject robot : robots.values()) {
				robot.draw(gl);
			}
		}
	}

	@Override
	public void onSurfaceChanged(GL10 gl, int width, int height) {
        gl.glViewport(0, 0, width, height);
        float ratio = (float) width / height;
        gl.glMatrixMode(GL10.GL_PROJECTION);
        gl.glLoadIdentity();
        gl.glFrustumf(ratio, -ratio, -1, 1, 3, 100);
	}

	@Override
	public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        gl.glEnable(GL10.GL_CULL_FACE);
        gl.glEnable(GL10.GL_DEPTH_TEST);
        // which is the front? the one which is drawn counter clockwise
        gl.glFrontFace(GL10.GL_CW);
        // which one should NOT be drawn
        gl.glCullFace(GL10.GL_BACK);
     
        gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
        gl.glEnableClientState(GL10.GL_COLOR_ARRAY);
        
        gl.glClearColor(0f, 0f, 0f, 1.0f);
        gl.glClearDepthf(1.0f);
        gl.glDepthFunc(GL10.GL_LEQUAL);
	}
}
