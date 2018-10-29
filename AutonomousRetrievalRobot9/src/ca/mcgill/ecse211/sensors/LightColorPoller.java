package ca.mcgill.ecse211.sensors;

import lejos.robotics.SampleProvider;

/**
 * Polls light sensor data for rgb colors
 */
public class LightColorPoller extends Thread {

	private SampleProvider lr;
	private DataController dataCont;
	private float[] ringData;

	private static int SLEEP_TIME = 50;

	public LightColorPoller(SampleProvider lr, float[] ringData, DataController dataCont) {
		this.lr = lr;
		this.dataCont = dataCont;
		this.ringData = ringData;
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		float[] light;
		while (true) {
			lr.fetchSample(ringData, 0); // acquire data
			light = (ringData); // extract from buffer, cast to int
			dataCont.setRGB(light); // now take action depending on value
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}
