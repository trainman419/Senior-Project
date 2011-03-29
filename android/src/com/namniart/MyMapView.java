/**
 * 
 */
package com.namniart;

import android.content.Context;
import android.util.AttributeSet;
import android.view.GestureDetector;
import android.view.MotionEvent;

import com.google.android.maps.MapView;

/**
 * @author hendrix
 *
 */
public class MyMapView extends MapView {
	
	private GestureDetector mGestureDetector;

	public MyMapView(Context context, AttributeSet attrs) {
		super(context, attrs);
		// TODO Auto-generated constructor stub
	}

	public MyMapView(Context context, AttributeSet attrs, int defStyle) {
		super(context, attrs, defStyle);
		// TODO Auto-generated constructor stub
	}

	public MyMapView(Context context, String apiKey) {
		super(context, apiKey);
		// TODO Auto-generated constructor stub
	}

	@Override
	public boolean onTouchEvent(MotionEvent e) {
		if( mGestureDetector != null ) mGestureDetector.onTouchEvent(e);
		return super.onTouchEvent(e);
	}
	
	public void setGestureDetector(GestureDetector d) {
		mGestureDetector = d;
	}
}