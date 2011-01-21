package edu.umich.soarrobot.SoarRobotTablet.layout;

import edu.umich.soarrobot.SoarRobotTablet.SoarRobotTablet;
import android.content.Context;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Paint.Style;
import android.graphics.Point;
import android.graphics.Rect;
import android.util.AttributeSet;
import android.util.Log;
import android.view.MotionEvent;
import android.view.SurfaceHolder;
import android.view.SurfaceHolder.Callback;
import android.view.SurfaceView;

public class MapView extends SurfaceView implements Callback {

	public static final String TAG = "MAP_VIEW";
	public static final int PX_PER_METER = 32;
	
	DrawThread dt;
	SoarRobotTablet activity;
	Point camera;
	float zoom;
	Point lastTouch;
	
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
		camera = new Point(0, 0);
		zoom = 1.0f;
		lastTouch = new Point(-1, -1);
	}
	
	public void setActivity(SoarRobotTablet activity) {
		this.activity = activity;
	}
	
	@Override
	public void surfaceChanged(SurfaceHolder holder, int format, int width,
			int height) {
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
			Point touch = new Point((int) event.getX(), (int) event.getY());
			if (lastTouch.x != -1 && lastTouch.y != -1) {
				synchronized (camera) {
					camera.x += touch.x - lastTouch.x;
					camera.y += touch.y - lastTouch.y;
				}
			}
			lastTouch = touch;
		} else if (action == MotionEvent.ACTION_DOWN) {
			lastTouch.x = (int) event.getX();
			lastTouch.y = (int) event.getY();
		} else if (action == MotionEvent.ACTION_CANCEL
				|| action == MotionEvent.ACTION_UP) {
			lastTouch.x = -1;
			lastTouch.y = -1;
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
			while(running) {
				Canvas c = null;
				try {
					c = sh.lockCanvas(null);
					synchronized (sh) {
						draw(c);
					}
				} finally {
					if (c != null)
						sh.unlockCanvasAndPost(c);
				}
				try {
					sleep(DELAY);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
		
		// marker coords
		float x = 0.0f;
		float y = 0.0f;
		float angle = 0.0f;
		float[] triangle = {0.0f, 0.0f, -0.5f, -1.0f,
				 -0.5f, -1.0f, 0.5f, -1.0f,
				 0.5f, -1.0f, 0.0f, 0.0f};
		
		// Map info
		float mapSize = 10.0f;
		private void draw(Canvas c) {

			x += 0.03f;
			x = x % mapSize;
			y += 0.07f;
			y = y % mapSize;
			angle += 1.0f;
			angle = angle % 360;
			
			Paint p = new Paint();
			
			p.setStyle(Style.FILL);
			p.setColor(Color.WHITE);
			c.drawRect(new Rect(0,0, c.getWidth(), c.getHeight()), p);
			
			int cx, cy;
			synchronized (camera) {
				cx = camera.x;
				cy = camera.y;
			}
			
			c.save();
			
			c.translate(cx, cy);
			c.scale(zoom, zoom);
			c.scale(PX_PER_METER, PX_PER_METER);
			
			p.setColor(Color.GRAY);
			for (int i = 0; i <= mapSize; ++i) {
				c.drawLine(0, i, mapSize, i, p);
				c.drawLine(i, 0, i, mapSize, p);
			}
			
			c.save();
			c.translate(x, y);
			c.rotate(angle);
			p.setColor(Color.RED);
			c.drawLines(triangle, p);
			p.setColor(Color.BLACK);
			p.setStyle(Style.STROKE);
			c.drawLines(triangle, p);
			c.restore();
			
			c.restore();
		}
	}
}
