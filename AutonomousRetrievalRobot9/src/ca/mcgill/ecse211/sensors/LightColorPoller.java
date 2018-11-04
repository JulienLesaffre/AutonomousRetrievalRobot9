package ca.mcgill.ecse211.sensors;

import lejos.robotics.SampleProvider;

/**
 * Polls light sensor data for rgb colors
 */
public class LightColorPoller extends Thread {

	private SampleProvider sampleProv;
	private ColorController cont;
	private float[] ringData;

	private static int SLEEP_TIME = 50;

	public LightColorPoller(SampleProvider sp, float[] ringData, ColorController cont) {
		this.sampleProv = sp;
		ringData = new float[sp.sampleSize()];
		this.cont = cont;
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		float[] data;
		while (true) {
			sampleProv.fetchSample(ringData, 0); // acquire data
			data = (ringData); // extract from buffer, cast to int
			cont.processRingData(data); // now take action depending on value
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}

}
