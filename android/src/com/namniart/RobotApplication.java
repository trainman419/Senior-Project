package com.namniart;

import android.app.Application;
import android.bluetooth.BluetoothDevice;

public class RobotApplication extends Application {

	private HardwareManager mHwMan;

	/**
	 * Called when application is created.
	 * 
	 * @see android.app.Application#onCreate()
	 */
	@Override
	public void onCreate() {
		super.onCreate();
		mHwMan = new StubHardwareManager();
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
		mHwMan = new StubHardwareManager();
	}

	/**
	 * Start a new HardwareManager with a new device. Stops the current HardwareManager if one is running.
	 * @param device The new bluetooth device to use
	 */
	public void startHwMan(BluetoothDevice device) {
		mHwMan.sendStop();
		mHwMan = new HardwareManager(device);
		mHwMan.start();
	}
}
