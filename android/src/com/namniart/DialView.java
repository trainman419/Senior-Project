package com.namniart;

import android.content.Context;
import android.content.res.TypedArray;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.graphics.RectF;
import android.util.AttributeSet;
import android.view.View;

public class DialView extends View {
	private float value;
	
	private int orientation;
	private int direction;
	private final int CW = 0;
	private final int CCW = 1;
	
	private int width;
	private int height;
	
	private Paint shadowPaint;
	private Paint bgPaint;
	private Paint linePaint;
	private Paint fillPaint;
	
	private static int shadow_size = 3;
	private static int corner_padding = 10;
	private RectF shadowBounds;
	private RectF arcBounds;
	private RectF bounds;
	
	private int pointer_x;
	private int pointer_y;
	private Path pointerPath;
	private float[] pointerPoints;
	
	private float base_angle;
	
	public DialView(Context context, AttributeSet attrs) {
		super(context);
		TypedArray a = context.getTheme().obtainStyledAttributes(
		        attrs,
		        R.styleable.DialView,
		        0, 0);

		try {
		    orientation = a.getInteger(R.styleable.DialView_position, 0);
		    direction = a.getInteger(R.styleable.DialView_direction, 0);
		} finally {
		    a.recycle();
		}
		value = 0.25f;

		shadowPaint = new Paint(0);
		shadowPaint.setStyle(Paint.Style.FILL);
		shadowPaint.setARGB(0x77, 0x20, 0x20, 0x20);
		shadowPaint.setAntiAlias(true);
		
		bgPaint = new Paint(0);
		bgPaint.setStyle(Paint.Style.FILL);
		bgPaint.setARGB(0x77, 0xa0, 0xa0, 0xa0);
		bgPaint.setAntiAlias(true);
		
		linePaint = new Paint(0);
		linePaint.setARGB(0xff, 0xff, 0, 0);
		linePaint.setAntiAlias(true);
		
		fillPaint = new Paint(0);
		fillPaint.setStyle(Paint.Style.FILL);
		fillPaint.setARGB(0xff, 0xc0, 0xc0, 0xc0);

		switch(orientation) {
		case 0:
			// top_left
			if( direction == CW ) base_angle = 90;
			else base_angle = 180;
			break;
		case 1:
			// top_right
			if( direction == CW ) base_angle = 180;
			else base_angle = 270;
			break;
		case 2:
			// bottom_left
			if( direction == CW ) base_angle = 0;
			else base_angle = 90;
			break;
		case 3:
		default:
			// bottom_right
			if( direction == CW ) base_angle = 270;
			else base_angle = 0;
			break;
		}
	}
	
	@Override
	protected void onSizeChanged(int w, int h, int oldw, int oldh) {
		// Account for padding
	    int xpad = getPaddingLeft() + getPaddingRight();
	    int ypad = getPaddingTop() + getPaddingBottom();
		width = w - xpad;
		height = h - ypad;

		switch(orientation) {
		case 0:
			// top_left
			shadowBounds = new RectF(-width + corner_padding * 2, -height + corner_padding * 2, width, height);
			pointer_x = corner_padding;
			pointer_y = corner_padding;
			break;
		case 1:
			// top_right
			shadowBounds = new RectF(0, -height + corner_padding * 2, width * 2 - corner_padding * 2, height);
			pointer_x = width - corner_padding;
			pointer_y = corner_padding;
			break;
		case 2:
			// bottom_left
			shadowBounds = new RectF(-width + corner_padding * 2, 0, width, height * 2 - corner_padding * 2);
			pointer_x = corner_padding;
			pointer_y = height - corner_padding;
			break;
		case 3:
		default:
			// bottom_right
			shadowBounds = new RectF(0, 0, width * 2 - corner_padding * 2, height * 2 - corner_padding * 2);
			pointer_x = width - corner_padding;
			pointer_y = height - corner_padding;
			break;
		}
		bounds = new RectF(shadowBounds.left + shadow_size, 
				shadowBounds.top + shadow_size, 
				shadowBounds.right - shadow_size,
				shadowBounds.bottom - shadow_size);

		float pointer_scale = (width - corner_padding - shadow_size)/corner_padding;
		pointerPath = new Path();
		pointerPath.moveTo(-1, 0);
		pointerPath.lineTo(0, -1 * pointer_scale );
		pointerPath.lineTo(1, 0);
		pointerPath.lineTo(0, 1);
		pointerPath.lineTo(-1, 0);
		
		float x[] = { -1,  0,  0, 1, 1,  0,  0, -1};
		float y[] = {  0, -1 * pointer_scale, -1 * pointer_scale, 0, 0, 1, 1, 0 };
		pointerPoints = new float[x.length*2];
		for(int i=0; i<x.length; i++) {
			pointerPoints[i*2] = x[i];
			pointerPoints[i*2+1] = y[i];
		}
	}
	
	@Override
	protected void onDraw(Canvas canvas) {
		super.onDraw(canvas);
		
		canvas.drawOval(shadowBounds, shadowPaint);
		canvas.drawOval(bounds, bgPaint);
		
		canvas.translate(pointer_x, pointer_y);
		canvas.scale(corner_padding, corner_padding);
		if( direction == CW ) {
			canvas.rotate(value * 90 + base_angle);
		} else {
			canvas.rotate(-value * 90 + base_angle);
		}
		canvas.drawPath(pointerPath, fillPaint);
		canvas.drawLines(pointerPoints, linePaint);
	}

}
