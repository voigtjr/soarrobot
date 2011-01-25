package edu.umich.soarrobot.SoarRobotTablet.layout;

import android.content.Context;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.widget.ScrollView;

public class NoTouchScroll extends ScrollView {

	public NoTouchScroll(Context context) {
		super(context);
	}

	public NoTouchScroll(Context context, AttributeSet attrs) {
		super(context, attrs);
	}

	public NoTouchScroll(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);
	}

	public boolean onTouchEvent(MotionEvent ev) {
		return true;
	}

	public boolean onInterceptTouchEvent(MotionEvent ev) {
		return false;
	}

}
