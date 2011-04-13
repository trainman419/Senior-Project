package com.namniart;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import android.app.AlertDialog;
import android.app.Dialog;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Intent;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.view.GestureDetector;
import android.view.Menu;
import android.view.MenuItem;
import android.view.MotionEvent;

import com.google.android.maps.GeoPoint;
import com.google.android.maps.MapActivity;
import com.google.android.maps.MapController;
import com.google.android.maps.MyLocationOverlay;
import com.google.android.maps.Overlay;
import com.google.android.maps.Projection;

public class RobotControl extends MapActivity implements SensorEventListener, GestureDetector.OnGestureListener, GestureDetector.OnDoubleTapListener {

	// instance variables
	RobotApplication mApp;
	
	// map-related variables
	private MyLocationOverlay mMyLoc;
	private MapController mMapController;
	private GestureDetector mGestureDetector;
	private LocationListOverlay mLocList;
	private MyMapView mMapView;
	private RobotLocationOverlay mLocOverlay;

	// bluetooth-related variables
	private BluetoothDevice mDevice;
	private ArrayList<BluetoothDevice> mDevices;
	
	// accelerometer-related variables
    private SensorManager mSensorManager;
    private Sensor mAccelerometer;

	// activity codes
	public static final int CHOOSE_DEVICE=1;
	
	/** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        
        // FIXME: make this work properly
        /*if( savedInstanceState != null ) {
	        Parcelable device = savedInstanceState.getParcelable("BluetoothDevice");
	        if( device != null && device instanceof BluetoothDevice ) {
	        	mDevice = (BluetoothDevice)device;
	        }
        }*/
        
        mApp = (RobotApplication)this.getApplication();
        
        setContentView(R.layout.main);
            	
        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY);

        mMapView = (MyMapView)findViewById(R.id.mapview);
		mMapView.setBuiltInZoomControls(true);
		mMapView.setSatellite(true);
		mMapView.setTraffic(false);
		
		mGestureDetector = new GestureDetector(mMapView.getContext(), this);
		mGestureDetector.setOnDoubleTapListener(this);
		mMapView.setGestureDetector(mGestureDetector);
		
		mMapController = mMapView.getController();

		mMyLoc = new MyLocationOverlay(this, mMapView);

		List<Overlay> overlays = mMapView.getOverlays();
		overlays.add(mMyLoc);
		
		// second-to-highest zoom level. comfortable for what we're doing
		mMapController.setZoom(18);
		
		mMyLoc.enableCompass();
		mMyLoc.enableMyLocation();
		mMyLoc.runOnFirstFix(new Runnable() {
			public void run() {
				mMapController.animateTo(mMyLoc.getMyLocation());
			}
		});
		
		// Green button from: http://commons.wikimedia.org/wiki/File:Button-Green.svg
		mLocList = new LocationListOverlay(getResources().getDrawable(R.drawable.button_green),
				mMapView.getContext(), mApp.getGPSQueue());
		overlays.add(mLocList);
		
		mLocOverlay = new RobotLocationOverlay(getResources().getDrawable(R.drawable.button_orange));
		overlays.add(mLocOverlay);
		
		mApp.addHandler('G', mLocOverlay);
    }

	@Override
	protected boolean isRouteDisplayed() {
		return false; // TODO: update this if/when we display a map
	}
	
	@Override
	protected boolean isLocationDisplayed() {
		return true;
	}
	
	private static final int CHOOSE_ID = Menu.FIRST;
	private static final int SHUTDOWN_ID = Menu.FIRST + 1;
	private static final int AUTONOMOUS_ID = Menu.FIRST + 2;
	private static final int RC_ID = Menu.FIRST + 3;
	private static final int STOP_ID = Menu.FIRST + 4;
	/**
	 * create the context menu for this Activity
	 */
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
    	super.onCreateOptionsMenu(menu);
    	menu.add(0, CHOOSE_ID, 0, R.string.bluetooth_picker);
    	menu.add(0, SHUTDOWN_ID, 0, R.string.shutdown);
    	menu.add(0, AUTONOMOUS_ID, 0, R.string.autonomous_mode);
    	menu.add(0, RC_ID, 0, R.string.rc_mode);
    	menu.add(0, STOP_ID, 0, R.string.bluetooth_stop);
    	return true;
    }

    /**
     * callback for selection of a menu item
     */
    @Override
    public boolean onMenuItemSelected(int featureId, MenuItem item) {
    	Intent i;
    	AlertDialog.Builder builder = new AlertDialog.Builder(this);
    	
    	builder.setMessage("Generic Message").setCancelable(false);
    	builder.setNeutralButton("Ok", null);
    	       
        switch (item.getItemId()) {
        case CHOOSE_ID:
        	mApp.stopHwMan();
        	Dialog dialog = ProgressDialog.show(this, "", "Looking for Bluetooth devices...", true);
        	BluetoothAdapter bt = BluetoothAdapter.getDefaultAdapter();
        	
	        if( bt != null ) {
		        Set<BluetoothDevice> devices = bt.getBondedDevices();
		        // display list of adapters; allow the user to pick one
		        if( devices != null && devices.isEmpty() ) {
		        	dialog.dismiss();
		        	dialog = builder.setMessage("No devices found; please pair a device with your phone.").create();
		        	dialog.show();
		        } else {		        	
		        	ArrayList<BluetoothDevice> list = new ArrayList<BluetoothDevice>();
		        	for( BluetoothDevice dev : devices ) {
		        		list.add(dev);
		        	}
		        	
		        	i = new Intent(this, BluetoothChooser.class);
		        	i.putExtra("devices", list);
		        	mDevices = list;
		        	dialog.dismiss();
		        	try {
		        		startActivityForResult(i, CHOOSE_DEVICE);
		        	} catch(Exception e) {
			        	dialog = builder.setMessage("Failed to start bluetooth chooser: " + e.getMessage()).create();
			        	dialog.show();
		        	}
		        }
	        } else {
	        	dialog.dismiss();
	        	dialog = builder.setMessage("No Bluetooth found").create();
	        	dialog.show();
	        }
        	return true;
        case SHUTDOWN_ID:
        	// send the shutdown command
        	mApp.getHwMan().setShutdown();
        	return true;
        case AUTONOMOUS_ID:
        	mApp.getHwMan().setAutonomous(true);
        	return true;
        case RC_ID:
        	mApp.getHwMan().setAutonomous(false);
        	return true;
        case STOP_ID:
        	// stop the HardwareManager
        	mApp.stopHwMan();
        	return true;        
        }
        return super.onMenuItemSelected(featureId, item);
    }

	/* this is called before onResume(), so we let onResume do the work of starting the worker thread
	 * @see android.app.Activity#onActivityResult(int, int, android.content.Intent)
	 */
	@Override
    protected void onActivityResult(int requestCode, int resultCode, Intent intent) {
        super.onActivityResult(requestCode, resultCode, intent);
        if(resultCode == RESULT_OK) {
	        switch(requestCode) {
	        case CHOOSE_DEVICE:
	        	if( mDevices != null) {
	        		mDevice = mDevices.get(intent.getIntExtra("index",0));
	        		mDevices = null; // free up our device list
	    			mApp.startHwMan(mDevice);
	        	}
	        	break;
	        }
        }
    }
		
	@Override
	protected void onPause() {
		super.onPause();
		//stopHwMan();
        mSensorManager.unregisterListener(this);
	}

	@Override
	protected void onResume() {
		super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_NORMAL);
	}
	
    /**
     * save our instance state, including our chosen bluetooth device
     */
    @Override
    protected void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        // FIXME: make this work properly
        outState.putParcelable("BluetoothDevice", mDevice);
    }
    
    /**
     * called when this activity is destroyed
     */
    @Override
    protected void onDestroy() {
    	mApp.removeHandler('G', mLocOverlay);
    }

    /* methods to implement SensorEventListener
     */

	@Override
	public void onAccuracyChanged(Sensor arg0, int arg1) {
		// TODO Auto-generated method stub
		// ignore this message
	}

	@Override
	public void onSensorChanged(SensorEvent arg0) {

		float a[] = new float[3];
		
		a[0] = arg0.values[0];
		a[1] = arg0.values[1];
		a[2] = arg0.values[2];
		
		/* interpreting the sensor data:
		 * (see: http://developer.android.com/reference/android/hardware/SensorEvent.html)
		 * vector direction components, when viewed in a normal orientation
		 * [0]: X: rightwards
		 * [1]: Y: up
		 * [2]: Z: out of screen, towards user
		 */
		/*Display d = null;
		switch(d.getRotation()) {
		// normal orientation
		case Surface.ROTATION_0:
			// do nothing
			break;
			
		case Surface.ROTATION_90:
			break;
			
		// upside-down
		case Surface.ROTATION_180:
			// swap up/down and left/right
			a[0] = -a[0];
			a[1] = -a[1];
			break;
			
		case Surface.ROTATION_270:
			break;
		}*/
		
		double forward_angle = (Math.atan2(a[2], a[1]) - Math.PI/2)/(Math.PI/4)*4*20;
		double lr_angle = (Math.atan2(Math.hypot(a[2], a[1]), a[0]) - Math.PI/2)/(Math.PI/4)*4*50;
		
		int steer = (int)lr_angle;
		if( steer > 100 ) steer = 100;
		if( steer < -100 ) steer = -100;
		
		byte s = (byte)steer;
		s += 120;
		mApp.getHwMan().setDirection(s);

		int speed = (int)forward_angle;
		if( speed > 100 ) speed = 100;
		if( speed < -100 ) speed = -100;
		byte m = (byte)speed;
		mApp.getHwMan().setSpeed(m);
	}

	@Override
	public boolean onDoubleTap(MotionEvent e) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean onDoubleTapEvent(MotionEvent e) {
		// TODO Auto-generated method stub
		if( e.getAction() == MotionEvent.ACTION_UP ) {
			Projection projection = mMapView.getProjection();
			float x = e.getX();
			float y = e.getY();
			System.out.println("Double-tap at (" + x + ", " + y +"); action: " + e.getAction());
			GeoPoint point = projection.fromPixels((int)x, (int)y);
			System.out.println("Point: " + point.toString());
			mLocList.addLocation(point);
		}
		return false;
	}

	@Override
	public boolean onSingleTapConfirmed(MotionEvent e) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean onDown(MotionEvent arg0) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean onFling(MotionEvent arg0, MotionEvent arg1, float arg2,
			float arg3) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void onLongPress(MotionEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean onScroll(MotionEvent e1, MotionEvent e2, float distanceX,
			float distanceY) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void onShowPress(MotionEvent e) {
		// TODO Auto-generated method stub
		
	}

	@Override
	public boolean onSingleTapUp(MotionEvent e) {
		// TODO Auto-generated method stub
		return false;
	}
    
}