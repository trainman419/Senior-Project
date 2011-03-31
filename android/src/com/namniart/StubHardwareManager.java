package com.namniart;

/**
 * A version of the HardwareManager that doesn't require a bluetooth device, and
 * doesn't use a helper thread. <br/>
 * <br/>
 * This class exists for testing, and to have something to call against when a bluetooth
 * device isn't connected. <br/> 
 * 
 * @author Austin Hendrix
 *
 */
public class StubHardwareManager extends HardwareManager {
	
	public StubHardwareManager(RobotApplication app) {
		super(null, app);
	}
	
	@Override
	public void run() {
		return; // do nothing and terminate if we are run.
	}
}
