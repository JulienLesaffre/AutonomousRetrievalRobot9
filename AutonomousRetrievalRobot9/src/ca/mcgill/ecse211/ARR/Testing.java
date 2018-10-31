package ca.mcgill.ecse211.ARR;

import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class Testing extends Thread{
	
	public Testing () throws OdometerExceptions {
		
	}
	
	public void run() {

		RingSet ringSetTesting;
		try {
			ringSetTesting = new RingSet();
			Thread ringSetTestingThread = new Thread(ringSetTesting);
			ringSetTestingThread.start();
		} catch (OdometerExceptions e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		
	}

}
