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
import android.graphics.Point;
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
    private static final float ZOOM_AMOUNT = 5.0f;
    private static float FRUSTUM_FRONT = 3.0f;
    
    // Variables that determine how the camera follows a robot
    private static final float FOLLOW_HEIGHT_FACTOR = 4.0f;
    private static final float FOLLOW_LOOK_AT_HEIGHT = 2.0f;
    
    private SoarRobotTablet activity;
    private PointF camera;
    private float zoom;
    private PointF lastTouch;
    private HashMap<Integer, SimObject> objects;
    private HashMap<String, SimObject> robots;
    private ArrayList<Rect> areas;
    private String nextObjectClass;
    private int follow;
    private Point windowSize; // The size of the window in pixel coordinates.
    private PointF frustumSize; // The size of the window in "real-world" coordinates.
        
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
        zoom = -40.0f;
        lastTouch = new PointF(-1.0f, -1.0f);
        objects = new HashMap<Integer, SimObject>();
        robots = new HashMap<String, SimObject>();
        areas = new ArrayList<Rect>();
        nextObjectClass = null;
        follow = robots.size();
        
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
    	PointF c = getCameraLocation();
    	PointF touch = new PointF(event.getX() / windowSize.x, 1.0f - event.getY() / windowSize.y);
    	if (c == camera) {
    		// TODO
    		// Given the origin of a ray (camera, zoom),
    		// the projection frustum (windowSize, FRUSTUM_FRONT),
    		// the direction of the frustum (downward),
    		// and the projection of the ray into the frustum
    		// (event.getX(), event.getY()):
    		// Find the (x,y) coordinates where the ray intersects
    		// z == 0.
    		
    	} else {
    		float cHeight = -zoom / FOLLOW_HEIGHT_FACTOR;
    		// TODO
    		// Given the origin of a ray (c, cHeight),
    		// the projection frustum (windowSize, FRUSTUM_FRONT),
    		// the direction of the frustum (camera, z=FOLLOW_LOOK_AT_HEIGHT)
    		// and the projection of the ray into the frustum
    		// (event.getX(), event.getY()):
    		// Find the (x,y) coordinates where the ray intersects
    		// z == 0.
    		
    		// The strategy will be to find the x/y/z coords of
    		// each of the four corners of the frustum, and then
    		// interpolate the touch coords between those points.
    		
    		// Start by finding the distance between c and camera.
    		float distCCam;
    		{
    			float dx = c.x - camera.x;
    			float dy = c.y - camera.y;
    			distCCam = (float) Math.sqrt(dx * dx + dy * dy);
    		}
    		
    		// Now use that distance, combined with the height of c,
    		// to find the angle from c to the object it's looking at.
    		// Keep in mind, the look-at target is actually
    		// FOLLOW_LOOK_AT_HEIGHT off the ground.
    		// NOTE: This is the up-down angle--NOT the angle
    		// as seen from above.
    		
    		float angleCCam;
    		{
    			float height = cHeight - FOLLOW_LOOK_AT_HEIGHT;
    			angleCCam = (float) Math.atan(distCCam / height);
    		}
    		
    		// Find the length from the bottom of the frustum to 
    		// the top, as viewed from above.
    		// (The width of the frustum will be unchanged)
    		
    		float lengthFrustumTop = (float) Math.cos(angleCCam) * frustumSize.y;
    		
    		// Find the distance from c of the bottom of the frustum and
    		// the top of the frustum, as seen from above.
    		
    		float distanceCenterFrustum = (float) Math.sin(angleCCam) * FRUSTUM_FRONT;
    		float distanceBottomFrustum = distanceCenterFrustum - lengthFrustumTop / 2.0f;
    		float distanceTopFrustum = distanceCenterFrustum + lengthFrustumTop / 2.0f;
    		float distanceIntersectFrustum = distanceBottomFrustum + touch.y * lengthFrustumTop;
    		
    		// Find the height of the bottom and top of the frustum.
    		// To do this, find the height of the center point of the frustum
    		// using angleCCam and FRUSTUM_FROM.
    		// Next, find the height of the rotated frustum.
    		// Then add (and subtract) half of that height to the height of the
    		// center point to find the upper (and lower) heights.
    		
    		float heightRotatedFrustrum = (float) Math.sin(angleCCam) * frustumSize.y;
    		float heightCenterFrustum = cHeight - (float) Math.cos(angleCCam) * FRUSTUM_FRONT;
    		float heightBottomFrustum = heightCenterFrustum - heightRotatedFrustrum / 2.0f;
    		float heightTopFrustum = heightCenterFrustum + heightRotatedFrustrum / 2.0f;
    		float heightIntersect = heightBottomFrustum + touch.y * heightRotatedFrustrum;
    		
    		// Okay, now we have distance from c and height for the top and bottom of the frustum.
    		// Let's use those to find the x,y,z coords for the four corners.
    		// Basically that means applying the rotation of c to the distance values
    		// and the width of the frustum.
    		
    		float rotation;
    		{
    			float dx = camera.x - c.x;
    			float dy = camera.y - c.y;
    			rotation = (float) Math.atan2(dy, dx);
    		}
    		
    		float closeRotation = (float) Math.atan((frustumSize.x / 2.0f) / distanceBottomFrustum);
    		float farRotation = (float) Math.atan((frustumSize.x / 2.0f) / distanceTopFrustum);
    		
    		float intersectX = (touch.x - 0.5f) * frustumSize.x;
    		float intersectRotation = - (float) Math.atan((intersectX) / distanceIntersectFrustum);
    		
    		PointF intersect;
    		{
    			float x = (float) Math.cos(rotation + intersectRotation) * distanceIntersectFrustum;
    			float y = (float) Math.sin(rotation + intersectRotation) * distanceIntersectFrustum;
    			intersect = new PointF(x, y);
    		}
    		
    		// Cool! Now we have the x,y,z coord of the intersection point.
    		// Use that to extrapolate where the ray intersects z = 0.
    		PointF floorTouch;
    		{
    			float dz = cHeight - heightIntersect;
    			float dzRatio = cHeight / dz;
    			floorTouch = new PointF(c.x + dzRatio * intersect.x, c.y + dzRatio * intersect.y);
    		}
    		
    		lastTouch = floorTouch;
    		
    	}
    	
    	/*
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
        */
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
        zoom += ZOOM_AMOUNT;
        draw();
    }

    public void zoomOut()
    {
        zoom -= ZOOM_AMOUNT;
        draw();
    }

    public void setNextClass(CharSequence nextClass)
    {
        nextObjectClass = nextClass.toString();
    }

	@Override
	public void onDrawFrame(GL10 gl) {
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
        gl.glMatrixMode(GL10.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		PointF c = getCameraLocation();
		if (c == camera) {
			GLU.gluLookAt(gl, c.x, c.y, zoom, camera.x, camera.y, 0.0f, 0.0f, 1.0f, 0.0f);			
		} else {
			GLU.gluLookAt(gl, c.x, c.y, zoom / FOLLOW_HEIGHT_FACTOR, camera.x, camera.y, -FOLLOW_LOOK_AT_HEIGHT, 0.0f, 0.0f, -1.0f);			
		}

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
		
		GLUtil.drawRect(gl, lastTouch.x, lastTouch.y, -0.01f, 0.5f, 0.5f, Color.RED);
	}

	@Override
	public void onSurfaceChanged(GL10 gl, int width, int height) {
        gl.glViewport(0, 0, width, height);
        float ratio = (float) width / height;
        gl.glMatrixMode(GL10.GL_PROJECTION);
        gl.glLoadIdentity();
        gl.glFrustumf(ratio, -ratio, -1, 1, FRUSTUM_FRONT, 1000);
        windowSize = new Point(width, height);
        frustumSize = new PointF(2.0f * ratio, 2.0f);
	}

	@Override
	public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        gl.glEnable(GL10.GL_CULL_FACE);
        gl.glEnable(GL10.GL_DEPTH_TEST);
        // which is the front? the one which is drawn clockwise
        gl.glFrontFace(GL10.GL_CW);
        // which one should NOT be drawn
        gl.glCullFace(GL10.GL_BACK);
     
        gl.glEnableClientState(GL10.GL_VERTEX_ARRAY);
        gl.glEnableClientState(GL10.GL_COLOR_ARRAY);
        
        gl.glClearColor(0f, 0f, 0f, 1.0f);
        gl.glClearDepthf(1.0f);
        gl.glDepthFunc(GL10.GL_LEQUAL);
	}
	
	/**
	 * Switch to the next thing to follow.
	 */
	public void toggleFollow() {
		follow = (follow + 1) % (robots.size() + 1);
	}
	
	/**
	 * Get the SimObject that this is following,
	 * or null if this isn't following anything.
	 * @return
	 */
	public SimObject getFollow() {
		if (follow >= robots.size()) {
			return null;
		}
		return (SimObject) robots.values().toArray()[follow];
	}
	
	private PointF getCameraLocation() {
		SimObject following = getFollow();
		if (following == null) {
			return camera;
		} else {
			camera = following.getLocation();
			float theta = (float) Math.toRadians(following.getTheta());
			PointF c = new PointF((float) (camera.x + Math.cos(theta) * zoom / 2.0f), (float) (camera.y + Math.sin(theta) * zoom / 2.0f));
			return c;
		}
	}
	
}
