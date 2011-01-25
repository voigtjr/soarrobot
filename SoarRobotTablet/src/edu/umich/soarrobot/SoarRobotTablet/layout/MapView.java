package edu.umich.soarrobot.SoarRobotTablet.layout;

import java.security.InvalidAlgorithmParameterException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;

import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import edu.umich.soarrobot.SoarRobotTablet.objects.SimObject;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Point;
import android.graphics.PointF;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.View;
import android.view.SurfaceHolder.Callback;
import android.view.View.OnClickListener;
import android.view.SurfaceView;
import android.widget.ToggleButton;

public class MapView extends SurfaceView implements Callback {

	public static final String TAG = "MAP_VIEW";
	public static final int PX_PER_METER = 32;
	
	private DrawThread dt;
	private SoarRobotTablet activity;
	private PointF camera;
	private float zoom;
	private PointF lastTouch;
	private HashMap<Integer, SimObject> objects;
	private ArrayList<ToggleButton> controlButtons;
	
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
		camera = new PointF(-2.0f, -2.0f);
		zoom = 1.0f;
		lastTouch = new PointF(-1.0f, -1.0f);
		objects = new HashMap<Integer, SimObject>();
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
	
	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width,	int height) {
		
	}

	@Override
	public void surfaceCreated(SurfaceHolder holder) {
		dt.setRunning(true);
		dt.start();
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
			}
			lastTouch = touch;
		} else if (action == MotionEvent.ACTION_DOWN) {
			PointF touch = new PointF(event.getX() / PX_PER_METER + camera.x, event.getY() / PX_PER_METER + camera.y);
			synchronized (objects) {
				for (int i = objects.size() - 1; i >= 0; --i) {
					SimObject obj = objects.get(i);
					if (obj.intersectsPoint(touch)) {
						activity.setSelectedObject(obj);
						break;
					}
				}
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
					sleep(DELAY);
				} catch (InterruptedException e) {
					e.printStackTrace();
				}
			}
		}
		
		// Map info
		float mapSize = 10.0f;
		
		private void draw(Canvas c) {
			
			Paint p = new Paint();
			
			p.setStyle(Style.FILL);
			p.setColor(Color.WHITE);
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
			
			p.setColor(Color.GRAY);
			for (int i = 0; i <= mapSize; ++i) {
				c.drawLine(0, i, mapSize, i, p);
				c.drawLine(i, 0, i, mapSize, p);
			}
			
			for (SimObject object : objects.values()) {
				c.save();
				object.draw(c, p);
				c.restore();
			}
			
			c.restore();
		}
	}
}
