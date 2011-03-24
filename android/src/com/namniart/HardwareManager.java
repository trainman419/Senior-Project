package com.namniart;

import java.io.InputStream;
import java.io.OutputStream;
import java.util.UUID;

import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;

/**
 * Interface thread to the robot hardware. Runs as a thread, receives periodic updates from the robot
 * and updates its internal state accordingly. <br/>
 * <br/>
 * General usage pattern: <br/>
 * Create an instance of the class, giving it a BluetoothDevice and a Handler to send output to. <br/>
 * call start(). <br/>
 * call getCurrentState() any time to get a copy of the current robot state <br/>
 */
/*
 * Serial communication protocol:
 * This is mostly documentation of the internals; users of this class don't have to read it.
 * 
 * Basis: the serial protocol focuses on piecemeal updates as new data becomes available from the robot
 * each update is preceded by an identifier, and followed by a null byte
 * 
 * Input sentences:
 *
 * Output sentences:
 * Speed: 'M[N]'
 * 	S: identifier
 * 	N: target speed (1 byte, signed)
 * 
 * Direction: 'D[N]'
 * 	D: identifier
 * 	N: steering setting (1 byte, signed, will require calibration)
 * 
 * 
 * @author Austin Hendrix
 *
 */

public class HardwareManager extends Thread {
	
	public static final byte MAX_SPEED = (byte)100;
   public static final byte MAX_HEADING = (byte)100;
	
	private BluetoothDevice mDevice;
	private boolean mStop;
	
	
	// robot output variables
	private Boolean mUpdateSent;
	private byte mSpeed;
	private byte mDirection;
	private boolean mShutdown;
		
	/**
	 * Create a hardware manager instance. Talk to the robot on the other of socket s, send status messages
	 * to master.
	 * @param d Bluetooth device to talk to
	 */
	public HardwareManager(BluetoothDevice d) {
		mDevice = d;
		mStop = false;
		
		mShutdown = false;
		mUpdateSent = false;
		mSpeed = 0;
		mDirection = 120;
	}
	
	private void message(String msg) {
		System.out.println(msg);
	}
	
	/**
	 * Run the hardware manager. Don't call this directly; invoke the superclass start() so that it runs
	 * as its own thread.
	 */
	@Override
	public void run() {
		BluetoothDevice dev = mDevice;
		try {
			
			// well-know UUID for SPP; from Android documentation: 
			// http://developer.android.com/reference/android/bluetooth/BluetoothDevice.html#createRfcommSocketToServiceRecord%28java.util.UUID%29
			BluetoothSocket socket = mDevice.createRfcommSocketToServiceRecord(UUID.fromString("00001101-0000-1000-8000-00805F9B34FB"));
			message("Created socket to " + mDevice.getName());

			// connect socket
			message("Connecting socket to " + dev.getName() + "...");
			socket.connect();
			message("Connected to " + dev.getName());
			
			// set up input and output streams
			OutputStream out = socket.getOutputStream();
			InputStream in = socket.getInputStream();
			int c = 0;
			int i;
			
			// main thread loop
			while( mStop != true ) {
				while( in.available() < 1 && mUpdateSent ) sleep(10); // this limits how quickly we can send/receive updates from the hardware
				//while( mUpdateSent && !mStop ) sleep(10);
				
				if( in.available() > 0 ) {
					//message("Receiving data");
					c = in.read();
					switch(c) {
					case 'S': // sonar message
						break;
					case 'L': // laser scan
						// read laser scan and throw it away, for now
						for( i=0; i<512; i++ ) {
							c = in.read();
						}
						do {
							c = in.read();
						} while( c != '\r' && c != '\n');
						//message("Received laser scan");
					default:
						// ignore unrecognized input
						break;
					}
				}
				
				if( !mUpdateSent ) {
					synchronized(mUpdateSent) {
						mUpdateSent = true;
						if( mShutdown ) {
							byte data[] = new byte[10];
							for( i=0; i<9; i++) data[i] = 'Z';
							data[9] = '\r';
							out.write(data);
						} else {
							byte data[] = new byte[3];
							
							data[0] = 'M';
							data[1] = mSpeed;
							data[2] = '\r';
							// speed command
							out.write(data);
							
							// direction command
							data[0] = 'S';
							data[1] = mDirection;
							data[2] = '\r';
							out.write(data);
						}
					}
					//message("Update sent");
				}
			}
			
			// hardcoded stop on thread end
			out.write('M');
			out.write(0);
			out.write('\r');
			out.write('S');
			out.write(120);
			out.write('\r');
			
			// don't forget to close our socket when we're done.
			socket.close();
			message("HardwareManager terminated");
			
		} catch(Exception e) {
			// tell the master why we died
			message("HardwareManager failure: " + e.getMessage());
		}
		return; // I like seeing where the end of my function is
	}
	
	/**
	 * Request that this thread stop.
	 */
	public void sendStop() {
		mStop = true;
	}
	

	/**
	 * Set the robot speed
	 * @param s target speed
	 */
	public void setSpeed(byte s) {
		synchronized (mUpdateSent) {
			mUpdateSent = false;
			mSpeed = s;
		}
	}
	
	public byte getSpeed() {
		return mSpeed;
	}
	
	/**
	 * Set the position of the robot's steering mechanism; positive is right
	 * @param dir target steering position in arbitrary units
	 */
	public void setDirection(byte dir) {
		synchronized (mUpdateSent) {
			mUpdateSent = false;
			mDirection = dir;
		}
	}
	
	public byte getDirection() {
		return mDirection;
	}
		
	public void fullStop() {
		synchronized (mUpdateSent) {
			mUpdateSent = false;
			mSpeed = 0;
			mDirection = 0;
		}
	}
	
	/**
	 * Send a shutdown command to the robot
	 */
	public void setShutdown() {
		synchronized(mUpdateSent) {
			mUpdateSent = false;
			mShutdown = true;
		}
	}
}

/* Simulator idea:
 * this will only really be useful for testing the mapping algorithms
 *
 * log entire state data to the SD card at regular intervals (every second?)
 * write a fake hardware module that plays back this log at the same interval
 * 
 * this has the severe limitation that the simulator won't react to commands,
 * but it should still be useful for passive mapping, and to replay obscure scenarios that
 * cause edge cases in the code so that they can be more carefully observed
 */