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

import java.awt.geom.Area;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.ConcurrentModificationException;
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
import android.util.Log;
import android.view.GestureDetector.OnDoubleTapListener;
import android.view.GestureDetector.OnGestureListener;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import edu.umich.robot.metamap.AbridgedAreaDescription;
import edu.umich.robot.metamap.AbridgedAreaDescriptions;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimArea;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimWall;

public class GLMapView extends GLSurfaceView implements Callback, Renderer, IMapView
{

    public static final String TAG = "MAP_VIEW";
    public static final int PX_PER_METER = 32;
    private static final float ZOOM_AMOUNT = 5.0f;
    private static float FRUSTUM_FRONT = 3.0f;
    
    // Variables that determine how the camera follows a robot
    private static final float FOLLOW_LOOK_AT_HEIGHT = 2.0f;

    private SoarRobotTablet activity;
    private PointF camera;
    private float zoom;
    private PointF lastFloorTouch;
    private PointF lastScreenTouch;
    private boolean fingerDown;
    private HashMap<Integer, SimObject> objects;
    private HashMap<String, SimObject> robots;
    private HashMap<Integer, SimArea> areas; // Map from id onto area.
    private ArrayList<SimWall> walls;
    private String nextObjectClass;
    private int follow; // Which robot to follow
    private boolean topDown;
    private Point windowSize; // The size of the window in pixel coordinates.
    private PointF frustumSize; // The size of the window in "real-world" coordinates.
    private float followHeightFactor;
    private float cameraOffsetX;
    private float cameraOffsetY;
    private float lastX;
    private long lastTouchDownTime;
    
    private FloatBuffer positionBuffer;

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
        lastFloorTouch = new PointF(-1.0f, -1.0f);
        lastScreenTouch = new PointF(-1.0f, -1.0f);
        objects = new HashMap<Integer, SimObject>();
        robots = new HashMap<String, SimObject>();
        areas = new HashMap<Integer, SimArea>();
        nextObjectClass = null;
        follow = robots.size();
        topDown = true;
        fingerDown = false;
        followHeightFactor = 4.0f;
        walls = new ArrayList<SimWall>();
        cameraOffsetX = 0.0f;
        cameraOffsetY = 0.0f;
        
        setRenderer(this);
        //setRenderMode(GLSurfaceView.RENDERMODE_WHEN_DIRTY);
    }

    public void setActivity(SoarRobotTablet activity)
    {
        this.activity = activity;
    }

    public void addObject(SimObject object)
    {
    	synchronized (objects) {
            objects.put(object.getID(), object);
		}
    }

    public void removeObject(SimObject object)
    {
    	synchronized (objects) {
            objects.remove(object.getID());
		}
    }

    public SimObject getObject(int objectID)
    {
    	synchronized (objects) {
    		return objects.get(objectID);
    	}
    }

    public void addRobot(SimObject robot)
    {
    	synchronized (robots) {
            robots.put(robot.getAttribute("name"), robot);
            ++follow;
		}
    }

    public void removeRobot(String name)
    {
    	synchronized (robots) {
            robots.remove(name);
		}
    }

    public void removeRobot(SimObject robot)
    {
    	synchronized (robots) {
            robots.remove(robot.getAttribute("name"));
            --follow;
            if (follow < 0) {
            	follow = 0;
            	topDown = true;
            }
		}
    }

    public SimObject getRobot(String name)
    {
    	synchronized (robots) {
            return robots.get(name);
		}
    }
    
	@Override
	public SimObject getSimObject(int id) {
		synchronized (objects) {
			return objects.get(id);
		}
	}

    public void draw()
    {
        requestRender();
    }
    
    @Override
    public boolean onTouchEvent(MotionEvent event)
    {
    	PointF screenTouch = new PointF(event.getX() / windowSize.x, 1.0f - event.getY() / windowSize.y);
    	if (screenTouch.x < 0.0f) {
    		screenTouch.x = 0.0f;
    	}
    	if (screenTouch.x > 1.0f) {
    		screenTouch.x = 1.0f;
    	}
    	if (screenTouch.y < 0.0f) {
    		screenTouch.y = 0.0f;
    	}
    	if (screenTouch.y >= 1.0f) {
    		screenTouch.y = 1.0f;
    	}
    	PointF floorTouch = floorTouch(screenTouch);
    	
    	int action = event.getAction();
        if (action == MotionEvent.ACTION_MOVE)
        {
			if (fingerDown) {
				if (topDown) {
					synchronized (camera) {
						camera.x -= (floorTouch.x - lastFloorTouch.x);
						camera.y -= (floorTouch.y - lastFloorTouch.y);
						floorTouch = floorTouch(screenTouch);
					}
				} else {
					if (event.getPointerCount() > 1) {
						followHeightFactor = 16.0f * (screenTouch.y + 0.1f);
					} else if (event.getPointerCount() == 1) {
					    float deltaHeight = (screenTouch.y - lastScreenTouch.y);
					    followHeightFactor += deltaHeight * followHeightFactor * 3.0f;
					    
					    if (followHeightFactor <= 1.0f) {
					        followHeightFactor = 1.0f;
					    }
					    Log.d("HeightFac", "value = " + followHeightFactor);
					    /*
					     * screenTouch is not being used because it is limited to 0 and 1
					     * which leads to undesirable results. getX() is used instead.
					     */
					    cameraOffsetX -= (event.getX() - lastX) / 200.0f;
	                    lastX = event.getX();
	                    if (cameraOffsetX > Math.PI * 2.0f) {
	                        cameraOffsetX -= Math.PI * 2.0f;
	                    } else if (cameraOffsetX < 0.0f) {
	                        cameraOffsetX += Math.PI * 2.0f;
	                    }
	                    //lastScreenTouch.y = screenTouch.y;
					}
				}
				if (event.getPointerCount() > 1) {
			    	PointF screenTouch2 = new PointF(event.getX(1) / windowSize.x, 1.0f - event.getY(1) / windowSize.y);
			    	float dx = screenTouch.x - screenTouch2.x;
			    	float dy = screenTouch.y - screenTouch2.y;
			    	zoom = -15.0f / (float) Math.sqrt(dx * dx + dy * dy) + 0.001f;
				}
			}
        }
        else if (action == MotionEvent.ACTION_DOWN)
        {
        	fingerDown = true;
        	lastX = event.getX();
        	
        	if (event.getEventTime() - lastTouchDownTime < 200) {
        	    resetCameraOffset();
        	}
        	lastTouchDownTime = event.getEventTime();
        	
            if (nextObjectClass != null)
            {
                activity.getRobotSession().sendMessage("object " + nextObjectClass + " " + floorTouch.x + " " + floorTouch.y);
                nextObjectClass = null;
            }
            else {
				try {
					boolean selected = false;
					synchronized (robots) {
						for (SimObject obj : robots.values()) {
							if (obj.intersectsPoint(floorTouch)) {
								activity.setSelectedObject(obj);
								selected = true;
								break;
							}
						}
					}
					if (!selected) {
						synchronized (objects) {

							for (SimObject obj : objects.values()) {
								if (obj.intersectsPoint(floorTouch, 1.0f)) {
									activity.setSelectedObject(obj);
									selected = true;
									break;
								}
							}
						}
					}
					draw();
				} catch (NullPointerException e) { // Don't know why this is happening
                    e.printStackTrace();
                }
            }

        }
        else if (action == MotionEvent.ACTION_CANCEL
                || action == MotionEvent.ACTION_UP)
        {
        	fingerDown = false;
        }
        synchronized (lastFloorTouch) {
        	lastFloorTouch = floorTouch;
		}
        synchronized (lastScreenTouch) {
        	lastScreenTouch = screenTouch;
        }
        return true;
    }

    /**
     * Find where the touch intersects the floor.
     * @param touch The touch on the screen, normalized
     * so that X and Y are between 0 and 1.
     * @return
     */
    private PointF floorTouch(PointF touch) {
    	PointF c = getCameraLocation();
    	synchronized (camera) {
    	if (c == camera) {
    		// Given the origin of a ray (camera, zoom),
    		// the projection frustum (windowSize, FRUSTUM_FRONT),
    		// the direction of the frustum (downward),
    		// and the projection of the ray into the frustum
    		// (event.getX(), event.getY()):
    		// Find the (x,y) coordinates where the ray intersects
    		// z == 0.
    		float dy = (touch.y - 0.5f) * frustumSize.y;
    		float dx = (touch.x - 0.5f) * frustumSize.x;
    		float factor = - zoom / FRUSTUM_FRONT;
    		dx *= factor;
    		dy *= factor;
    		return new PointF(c.x + dx, c.y + dy);
       	} else {
    		float cHeight = -zoom / followHeightFactor;
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
    		return floorTouch;
    	}
		}
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
                areas.put(aad.id, new SimArea(aad.id, new Rect(left, top, right, bottom), aad.type));
			}
        }
        makeWalls();
    }

    private void makeWalls() {
    	Point max = new Point(0, 0);
    	boolean [][] open;
		synchronized (areas) {
			for (SimArea area : areas.values())
			{
				Rect r = area.getRect();
				if (r.bottom < max.y)
				{
					max.y = r.bottom;
				}
				if (r.right > max.x)
				{
					max.x = r.right;
				}
			}
			open = new boolean[max.x][-max.y];
			for (SimArea area : areas.values())
			{
				Rect r = area.getRect();
				for (int x = r.left; x < r.right; ++x)
				{
					for (int y = r.top; y > r.bottom; --y)
					{
						open[x][-y] = true;
					}
				}
			}
		}
		synchronized (walls) {
			walls.clear();
			for (int x = 0; x + 1 < max.x; ++x)
			{
				for (int y = 0; y - 1 > max.y; --y)
				{
					// Check to the right and to the bottom.
					if (open[x][-y] != open[x+1][-y])
					{
						walls.add(new SimWall(new Point(x + 1, y - 1), new Point(x + 1, y)));
					}
					if (open[x][-y] != open[x][-(y-1)])
					{
						walls.add(new SimWall(new Point(x, y - 1), new Point(x + 1, y - 1)));
					}
				}
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

    public void resetCameraOffset() {
        cameraOffsetX = 0.0f;
        cameraOffsetY = 0.0f;
    }
    
    public void setNextClass(CharSequence nextClass)
    {
        nextObjectClass = nextClass.toString();
    }

	@Override
	public void onDrawFrame(GL10 gl) {
		try {
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);
        gl.glMatrixMode(GL10.GL_MODELVIEW);
		gl.glLoadIdentity();
		
		if (positionBuffer != null)
		{
			gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_POSITION, positionBuffer);
		}
		
		synchronized (camera) {
			PointF c = getCameraLocation();
			if (c == camera) {
				GLU.gluLookAt(gl, c.x, c.y, zoom, camera.x, camera.y, 0.0f, 0.0f, 1.0f, 0.0f);			
			} else {
				GLU.gluLookAt(gl, c.x, c.y, zoom / followHeightFactor, camera.x, camera.y, -FOLLOW_LOOK_AT_HEIGHT, 0.0f, 0.0f, -1.0f);			
			}
		}

		synchronized (areas) {
			for (SimArea area : areas.values()) {
				area.draw(gl);
			}
		}
		
		synchronized (walls) {
			for (SimWall wall : walls)
			{
				wall.draw(gl);
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
		
		/*
		synchronized (lastFloorTouch) {
			GLUtil.drawRect(gl, lastFloorTouch.x, lastFloorTouch.y, -0.01f, 0.5f, 0.5f, Color.GRAY);
		}
		*/

		} catch (ConcurrentModificationException e) {
			e.printStackTrace();
		}
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
        gl.glEnableClientState(GL10.GL_NORMAL_ARRAY);
        
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        gl.glClearDepthf(1.0f);
        gl.glDepthFunc(GL10.GL_LEQUAL);

        // lighting
        /*
        gl.glEnable(GL10.GL_LIGHTING);
        gl.glEnable(GL10.GL_COLOR_MATERIAL);
        
        float[] ambientLight = {0.5f, 0.5f, 0.5f, 1.0f};
        ByteBuffer bb = ByteBuffer.allocateDirect(ambientLight.length * 4);
        bb.order(ByteOrder.nativeOrder());
        FloatBuffer ambientBuffer = bb.asFloatBuffer();
        ambientBuffer.put(ambientLight);
        ambientBuffer.position(0);
        gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_AMBIENT, ambientBuffer);
        
        float[] diffuseLight = {1.0f, 1.0f, 1.0f, 1.0f};
        bb = ByteBuffer.allocateDirect(diffuseLight.length * 4);
        bb.order(ByteOrder.nativeOrder());
        FloatBuffer diffuseBuffer = bb.asFloatBuffer();
        ambientBuffer.put(diffuseLight);
        gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_DIFFUSE, diffuseBuffer);
        
        float[] specularLight = {0.0f, 0.0f, 0.0f, 1.0f};
        bb = ByteBuffer.allocateDirect(diffuseLight.length * 4);
        bb.order(ByteOrder.nativeOrder());
        FloatBuffer specularBuffer = bb.asFloatBuffer();
        specularBuffer.put(specularLight);
        specularBuffer.position(0);
        gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_SPECULAR, specularBuffer);
        
        float[] lightPosition = {1.0f, 1.0f, 1.0f, 1.0f};
        bb = ByteBuffer.allocateDirect(lightPosition.length * 4);
        bb.order(ByteOrder.nativeOrder());
        positionBuffer = bb.asFloatBuffer();
        positionBuffer.put(lightPosition);
        positionBuffer.position(0);
        gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_POSITION, positionBuffer);
        
        FloatBuffer mab = ByteBuffer.allocateDirect(4 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mab.put(new float[]{1.0f, 1.0f, 1.0f, 1.0f});
        mab.position(0);
        FloatBuffer mdb = ByteBuffer.allocateDirect(4 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        mdb.put(new float[]{1.0f, 1.0f, 1.0f, 1.0f});
        mdb.position(0);
        FloatBuffer msb = ByteBuffer.allocateDirect(4 * 4).order(ByteOrder.nativeOrder()).asFloatBuffer();
        msb.put(new float[]{0.0f, 0.0f, 0.0f, 1.0f});
        msb.position(0);
        
        gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_AMBIENT, mab);
        gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_DIFFUSE, mdb);
        gl.glMaterialfv(GL10.GL_FRONT_AND_BACK, GL10.GL_SPECULAR, msb);
        gl.glMaterialf(GL10.GL_FRONT_AND_BACK, GL10.GL_SHININESS, 0.0f);
        
        gl.glEnable(GL10.GL_LIGHT0);
        
		gl.glShadeModel(GL10.GL_SMOOTH); 			//Enable Smooth Shading
		*/
        /*
        float[] lightDirection = {0.0f, 0.0f, -1.0f};
        FloatBuffer directionBuffer = FloatBuffer.allocate(lightDirection.length);
        directionBuffer.put(lightDirection);
        directionBuffer.position(0);
        gl.glLightfv(GL10.GL_LIGHT0, GL10.GL_SPOT_DIRECTION, directionBuffer);
        gl.glLightf(GL10.GL_LIGHT0, GL10.GL_SPOT_CUTOFF, 45.0f);
        */
	}
	
	/**
	 * Switch to the next thing to follow.
	 */
	public void toggleFollow() {
		synchronized (robots) {
			follow = (follow + 1) % (robots.size() + 1);
			topDown = follow == robots.size();
		}
	}
	
	/**
	 * Get the SimObject that this is following,
	 * or null if this isn't following anything.
	 * @return
	 */
	public SimObject getFollow() {
		synchronized (robots) {
			if (follow >= robots.size()) {
				return null;
			}
			return (SimObject) robots.values().toArray()[follow];
		}
	}
	
	private PointF getCameraLocation() {
		SimObject following = getFollow();
		synchronized (camera) {
			if (following == null) {
				return camera;
			} else {
				camera = following.getLocation();
				float theta = (float) Math.toRadians(following.getTheta());
				PointF c = new PointF((float) (camera.x + (Math.cos(theta + cameraOffsetX))
						* zoom / 2.0f), (float) (camera.y + (Math.sin(theta + cameraOffsetX))
						* zoom / 2.0f));
				return c;
			}
		}
	}
	
	@Override
	public void pickUpObject(String robotName, int id) {
		SimObject robot = robots.get(robotName);
		SimObject pickedUp = objects.get(id);
		robot.setCarrying(pickedUp);
		pickedUp.setVisible(false);
	}
	
	@Override
	public void dropObject(String robotName) {
		SimObject robot = robots.get(robotName);
		SimObject dropped = robot.getCarrying();
		if (dropped != null) {
			dropped.setVisible(true);
			robot.setCarrying(null);
		}
	}

	@Override
	public void doorClose(int id) {
		SimArea a = areas.get(id);
		if (a == null) {
			return;
		}
		a.setDoorClosed(true);
	}

	@Override
	public void doorOpen(int id) {
		SimArea a = areas.get(id);
		if (a == null) {
			return;
		}
		a.setDoorClosed(false);
	}

	@Override
	public void roomLight(int id, boolean on) {
		SimArea a = areas.get(id);
		if (a == null) {
			return;
		}
		a.setLightsOn(on);
	}
}