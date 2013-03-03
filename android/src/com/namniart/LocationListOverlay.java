package com.namniart;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.OverlayItem;

public class LocationListOverlay extends ItemizedOverlay<OverlayItem> {
	private GPSQueue mItems;
	private Context mContext;
	
	public LocationListOverlay(Drawable d, Context c, GPSQueue q) {
		super(boundCenter(d));
		mContext = c;
		mItems = q;
		populate();
	}
	
	@Override
	public int size() {
		return mItems.size();
	}
	
	@Override
	protected OverlayItem createItem(int i) {
		return new OverlayItem(mItems.get(i), Integer.toString(i), Integer.toString(i));
	}
	
	public void update() {
		populate();
	}
	
	public void addLocation(GeoPoint p) {
		mItems.add(p);
		populate();
	}
	
	@Override
	protected boolean onTap(final int index) {
		System.out.println("Tap on index: " + index);
		
		AlertDialog.Builder dialog = new AlertDialog.Builder(mContext);
		dialog.setTitle("Options");
		dialog.setMessage("Point " + index);
		dialog.setNegativeButton("Cancel", null);
		dialog.setPositiveButton("Delete", new DialogInterface.OnClickListener() {
			@Override
			public void onClick(DialogInterface dialog, int which) {
				mItems.remove(index);
				setLastFocusedIndex(-1);
				populate();
			}
		});
		dialog.show();
		return true;
	}
}
