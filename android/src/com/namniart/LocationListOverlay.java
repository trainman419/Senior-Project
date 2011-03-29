package com.namniart;

import java.util.ArrayList;
import java.util.List;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;
import android.graphics.drawable.Drawable;
import android.location.Location;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.OverlayItem;

public class LocationListOverlay extends ItemizedOverlay<OverlayItem> {
	private List<Location> mItems;
	private Context mContext;
	
	public LocationListOverlay(Drawable d, Context c) {
		super(boundCenter(d));
		mContext = c;
		mItems = new ArrayList<Location>();
		populate();
	}
	
	@Override
	public int size() {
		return mItems.size();
	}
	
	@Override
	protected OverlayItem createItem(int i) {
		int lat, lon;
		lat = (int)(mItems.get(i).getLatitude() * 1E6);
		lon = (int)(mItems.get(i).getLongitude() * 1E6);
		return new OverlayItem(new GeoPoint(lat, lon), Integer.toString(i), Integer.toString(i));
	}
	
	public void update() {
		populate();
	}
	
	public void addLocation(Location l) {
		mItems.add(l);
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
				// TODO trigger an update to send this data to the robot
				mItems.remove(index);
				setLastFocusedIndex(-1);
				populate();
			}
		});
		dialog.show();
		return true;
	}
}
