/**
 * 
 */
package com.namniart;

import android.graphics.drawable.Drawable;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.ItemizedOverlay;
import com.google.android.maps.OverlayItem;

/**
 * @author hendrix
 *
 */
public class RobotLocationOverlay extends ItemizedOverlay<OverlayItem> implements PacketHandler {
	
	private GeoPoint mLocation;

	public RobotLocationOverlay(Drawable defaultMarker) {
		super(boundCenter(defaultMarker));
		mLocation = new GeoPoint(0, 0);
		populate();
	}

	@Override
	protected OverlayItem createItem(int i) {
		return new OverlayItem(mLocation, "Robot", "Robot");
	}

	@Override
	public int size() {
		return 1;
	}

	@Override
	public void handlePacket(Packet p) {
		int lat = p.reads32();
		int lon = p.reads32();
		System.out.println("Got location. Lat: " + lat + ", lon: " + lon + " (" + p.sz() + " remaining)");
		mLocation = new GeoPoint(lat, lon);
		populate();
	}
}
