package edu.umich.soarrobot.SoarRobotTablet.layout;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.PointF;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import android.view.SurfaceView;
import edu.umich.robot.metamap.AbridgedAreaDescription;
import edu.umich.robot.metamap.AbridgedAreaDescriptions;
import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;

public class MapView extends SurfaceView implements Callback {
	
	public static final String TAG = "MAP_VIEW";
	public static final int PX_PER_METER = 32;
	private static final float zoomRate = 1.5f;
	
	private DrawThread dt;
	private SoarRobotTablet activity;
	private PointF camera;
	private float zoom;
	private PointF lastTouch;
	private HashMap<Integer, SimObject> objects;
	private HashMap<String, SimObject> robots;
	private ArrayList<Rect> areas;
	
	public MapView(Context context) {
		super(context);
		init();
	}

	public MapView(Context context, AttributeSet attrs) {
		super(context, attrs);
		init();
	}

	public MapView(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);
		init();
	}
	
	private void init() {
		SurfaceHolder sh = getHolder();
		sh.addCallback(this);
		dt = new DrawThread(sh);
		camera = new PointF(0.0f, 0.0f);
		zoom = 1.0f;
		lastTouch = new PointF(-1.0f, -1.0f);
		objects = new HashMap<Integer, SimObject>();
		robots = new HashMap<String, SimObject>();
		areas = new ArrayList<Rect>();
	}
	
	public void setActivity(SoarRobotTablet activity) {
		this.activity = activity;
	}
	
	public void addObject(SimObject object) {
		objects.put(object.getID(), object);
	}
	
	public void removeObject(SimObject object) {
		objects.remove(object.getID());
	}
	
	public SimObject getObject(int objectID) {
		return objects.get(objectID);
	}
	
	public void addRobot(SimObject robot) {
		robots.put(robot.getAttribute("name"), robot);
	}
	
	public void removeRobot(String name) {
		robots.remove(name);
	}
	
	public void removeRobot(SimObject robot) {
		robots.remove(robot.getAttribute("name"));
	}
	
	public SimObject getRobot(String name) {
		return robots.get(name);
	}
	
	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width,	int height) {
		
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		dt.setRunning(true);
		dt.start();
	}
	
	public void draw() {
		if (dt != null) {
			synchronized (dt) {
				dt.notify();
			}
		}
	}

	@Override
	public void surfaceDestroyed(SurfaceHolder holder) {
		dt.setRunning(false);
		boolean retry = true;
		while (retry) {
			try {
				dt.join();
				retry = false;
			} catch (InterruptedException e) {
			}
		}
	}
	
	@Override
	public boolean onTouchEvent(MotionEvent event) {
		int action = event.getAction();
		if (action == MotionEvent.ACTION_MOVE) {
			PointF touch = new PointF(event.getX(), event.getY());
			if (lastTouch.x >= 0.0f && lastTouch.y >= 0.0f) {
				synchronized (camera) {
					camera.x -= (touch.x - lastTouch.x) / PX_PER_METER;
					camera.y -= (touch.y - lastTouch.y) / PX_PER_METER;
				}
				draw();
			}
			lastTouch = touch;
		} else if (action == MotionEvent.ACTION_DOWN) {
			PointF touch = new PointF((event.getX() / (PX_PER_METER) + camera.x) / zoom, (event.getY() / (PX_PER_METER) + camera.y) / zoom);
			try {
				boolean selected = false;
				synchronized (objects) {
					for (SimObject obj : robots.values()) {
						if (obj.intersectsPoint(touch)) {
							activity.setSelectedObject(obj);
							selected = true;
							break;
						}
					}
					if (!selected) {
						for (SimObject obj : objects.values()) {
							if (obj.intersectsPoint(touch)) {
								activity.setSelectedObject(obj);
								selected = true;
								break;
							}
						}
					}
				}
				if (selected) {
				    draw();
				}
			} catch (NullPointerException e) { // Don't know why this is happening
				e.printStackTrace();
			}
			lastTouch.x = (int) event.getX();
			lastTouch.y = (int) event.getY();
		} else if (action == MotionEvent.ACTION_CANCEL
				|| action == MotionEvent.ACTION_UP) {
			lastTouch.x = -1.0f;
			lastTouch.y = -1.0f;
		}
		return true;
	}
	
	private class DrawThread extends Thread {
		
		private SurfaceHolder sh;
		boolean running;
		private static final long FPS = 60;
		private static final long DELAY = 1000 / FPS;
		
		public DrawThread(SurfaceHolder sh) {
			this.sh = sh;
		}
		
		void setRunning(boolean running) {
			this.running = running;
		}
		
		@Override
		public void run() {
			try {
			while (running) {
				Canvas c = null;
				synchronized (sh) {
					try {
						c = sh.lockCanvas(null);
						draw(c);
					} finally {
						if (c != null)
							sh.unlockCanvasAndPost(c);
					}
				}
				try {
					synchronized (this) {
						this.wait();
					}
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
			} catch (NullPointerException e) {
				e.printStackTrace();
			}
		}	public void zoomIn() {

		}
		
		// Map info
		float mapSize = 10.0f;
		
		private void draw(Canvas c) {
			
			Paint p = new Paint();
			
			p.setStyle(Style.FILL);
			p.setColor(Color.BLACK);
			c.drawRect(new Rect(0,0, c.getWidth(), c.getHeight()), p);
			
			float cx, cy;
			synchronized (camera) {
				cx = camera.x;
				cy = camera.y;
			}
			
			c.save();
			
			c.scale(PX_PER_METER, PX_PER_METER);
			c.translate(-cx, -cy);
			c.scale(zoom, zoom);
			
			p.setColor(Color.WHITE);
			for (Rect r : areas) {
				c.save();
				c.translate(r.left, r.top);
				c.drawRect(0, 0, r.width(), r.height(), p);
				c.restore();
			}
			
			for (SimObject object : objects.values()) {
				c.save();
				object.draw(c, p);
				c.restore();
			}
			
			for (SimObject robot : robots.values()) {
				c.save();
				robot.draw(c, p);
				c.restore();
			}
			
			c.restore();
		}
	}

	public void deserializeMap(String map) {
		String[] areaList = map.split(";");
		for (String area : areaList) {
			if (area.trim().length() == 0) {
				continue;
			}
			System.out.println("Parsing area:\n" + area);
			AbridgedAreaDescription aad = AbridgedAreaDescriptions.parseArea(area);
			List<Double> xywh = aad.xywh;
			int x = (int)(double)xywh.get(0);
			int y = (int)(double)xywh.get(1);
			areas.add(new Rect(x, y, x + (int)(double)xywh.get(2), y + (int)(double)xywh.get(3)));
		}
	}

	public void zoomIn() {
		zoom = zoom * zoomRate;
		draw();
	}
	
	public void zoomOut() {
		zoom = zoom / zoomRate;
		draw();
	}
}
