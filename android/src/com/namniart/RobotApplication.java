package com.namniart;

import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import android.app.Application;
import android.bluetooth.BluetoothDevice;

public class RobotApplication extends Application {

	private HardwareManager mHwMan;
	private GPSQueue mQueue;
	private Map<Integer, List<PacketHandler>> mHandlers;

	/**
	 * Called when application is created.
	 * 
	 * @see android.app.Application#onCreate()
	 */
	@Override
	public void onCreate() {
		super.onCreate();
		mHwMan = new StubHardwareManager(this);
		mQueue = new GPSQueue(this);
		mHandlers = new HashMap<Integer, List<PacketHandler>>();
	}
	
	/**
	 * Called when application is terminated
	 * 
	 * @see android.app.Application#onTerminate()
	 */
	@Override
	public void onTerminate() {
		super.onTerminate();
		mHwMan.sendStop();
	}
	
	/**
	 * Get the current HardwareManager. guaranteed not to be null.
	 * @return the current HardwareManager
	 */
	public HardwareManager getHwMan() {
		return mHwMan;
	}
	
	/**
	 * Stop the current HardwareManager
	 */
	public void stopHwMan() {
		mHwMan.sendStop();
		mHwMan = new StubHardwareManager(this);
	}

	/**
	 * Start a new HardwareManager with a new device. Stops the current HardwareManager if one is running.
	 * @param device The new bluetooth device to use
	 */
	public void startHwMan(BluetoothDevice device) {
		mHwMan.sendStop();
		mHwMan = new HardwareManager(device, this);
		mHwMan.start();
	}
	
	/**
	 * Get the instance of the GPS Queue
	 */
	public GPSQueue getGPSQueue() {
		return mQueue;
	}
	
	/**
	 * Add a handler for a particular type of packet
	 */
	public void addHandler(int type, PacketHandler p) {
		if( !mHandlers.containsKey(type) ) {
			mHandlers.put(type, new LinkedList<PacketHandler>());
		}
		mHandlers.get(type).add(p);
	}
	
	/**
	 * Remove a handler for a particular type of packet
	 */
	public void removeHandler(int type, PacketHandler p) {
		if( mHandlers.containsKey(type) ) {
			mHandlers.get(type).remove(p);
			// don't delete the list, even if it's empty, because we're very likely to reuse it
		}
	}
	
	/**
	 * Get the packet handlers for a particular type of packet
	 * @param type: The type of packet
	 * @return a list of handlers
	 */
	public List<PacketHandler> getHandlers(int type) {
		return mHandlers.get(type);
	}
}
