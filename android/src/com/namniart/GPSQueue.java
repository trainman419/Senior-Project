package com.namniart;

import java.util.LinkedList;
import java.util.List;

import com.google.android.maps.GeoPoint;

// TODO: update for new diff-based location protocol
public class GPSQueue {
	private int cursor;
	private List<GeoPoint> points;
	private RobotApplication mApp;
	
	public GPSQueue(RobotApplication app) {
		cursor = 0;
		points = new LinkedList<GeoPoint>();
		mApp = app;
	}

	public int getCursor() {
		return cursor;
	}
	
	public void setCursor(int c) {
		cursor = c;
	}
	
	private Packet pointPacket(GeoPoint p) {
		Packet res = new Packet('L');
		res.append((byte)1); // APPEND
		res.append(p.getLatitudeE6());
		res.append(p.getLongitudeE6());
		res.finish();
		
		return res;
	}
	
	private Packet deletePacket(int pos) {
		Packet res = new Packet('L');
		res.append((byte)2); // DELETE
		res.append(pos);
		res.finish();
		
		return res;
	}
	
	private void transmit(final Packet p) {
		new Thread() {
			public void run() {
				mApp.getHwMan().sendPacket(p);
			}
		}.start();
	}
	
	public boolean add(GeoPoint arg0) {
		boolean r = points.add(arg0);
		transmit(pointPacket(arg0));
		return r;
	}

	public GeoPoint get(int location) {
		return points.get(location);
	}

	public GeoPoint remove(int location) {
		GeoPoint r = points.remove(location);
		transmit(deletePacket(location));
		return r;
	}

	public int size() {
		return points.size();
	}
}
