package com.namniart;

import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.ListIterator;

import com.google.android.maps.GeoPoint;

public class GPSQueue implements List<GeoPoint> {
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
	
	private Packet toPacket() {
		Packet res = new Packet('L');
		
		//res.append((byte)'L');
		res.append((byte)points.size());
		res.append((byte)cursor);
		
		for( GeoPoint p : points ) {
			res.append(p.getLatitudeE6());
			res.append(p.getLongitudeE6());
		}
		res.finish();
		
		return res;
	}
	
	private void transmit() {
		new Thread() {
			public void run() {
				//mApp.getHwMan().sendBytes(toByteArray());
				mApp.getHwMan().sendPacket(toPacket());
			}
		}.start();
	}
	
	@Override
	public boolean add(GeoPoint arg0) {
		boolean r = points.add(arg0);
		transmit();
		return r;
	}

	@Override
	public void add(int arg0, GeoPoint arg1) {
		points.add(arg0, arg1);
		// TODO: deal with cursor
		transmit();
	}

	@Override
	public boolean addAll(Collection<? extends GeoPoint> arg0) {
		boolean r = points.addAll(arg0);
		transmit();
		return r;
	}

	@Override
	public boolean addAll(int arg0, Collection<? extends GeoPoint> arg1) {
		boolean r = points.addAll(arg0, arg1);
		transmit();
		return r;
	}

	@Override
	public void clear() {
		points.clear();
		cursor = 0;
		transmit();
	}

	@Override
	public boolean contains(Object arg0) {
		return points.contains(arg0);
	}

	@Override
	public boolean containsAll(Collection<?> arg0) {
		return points.containsAll(arg0);
	}

	@Override
	public GeoPoint get(int location) {
		return points.get(location);
	}

	@Override
	public int indexOf(Object object) {
		return points.indexOf(object);
	}

	@Override
	public boolean isEmpty() {
		return points.isEmpty();
	}

	@Override
	public Iterator<GeoPoint> iterator() {
		return points.iterator();
	}

	@Override
	public int lastIndexOf(Object object) {
		return points.lastIndexOf(object);
	}

	@Override
	public ListIterator<GeoPoint> listIterator() {
		return points.listIterator();
	}

	@Override
	public ListIterator<GeoPoint> listIterator(int location) {
		return points.listIterator(location);
	}

	@Override
	public GeoPoint remove(int location) {
		// TODO deal with cursor properly
		GeoPoint r = points.remove(location);
		transmit();
		return r;
	}

	@Override
	public boolean remove(Object object) {
		// TODO deal with cursor properly
		boolean r = points.remove(object);
		transmit();
		return r;
	}

	@Override
	public boolean removeAll(Collection<?> arg0) {
		boolean r = points.removeAll(arg0);
		transmit();
		return r;
	}

	@Override
	public boolean retainAll(Collection<?> arg0) {
		boolean r = points.retainAll(arg0);
		transmit();
		return r;
	}

	@Override
	public GeoPoint set(int location, GeoPoint object) {
		GeoPoint r = points.set(location, object);
		transmit();
		return r;
	}

	@Override
	public int size() {
		return points.size();
	}

	@Override
	public List<GeoPoint> subList(int start, int end) {
		return points.subList(start, end);
	}

	@Override
	public Object[] toArray() {
		return points.toArray();
	}

	@Override
	public <T> T[] toArray(T[] array) {
		return points.toArray(array);
	}
}
