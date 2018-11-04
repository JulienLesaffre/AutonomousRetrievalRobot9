package ca.mcgill.ecse211.sensors;

import lejos.robotics.SampleProvider;

/**
 * The while loop at the bottom executes in a loop. Assuming that the us.fetchSample, and cont.processUSData
 * methods operate in about 20mS, and that the thread sleeps for 50 mS at the end of each loop, then
 * one cycle through the loop is approximately 70 mS. This corresponds to a sampling rate of 1/70mS
 * or about 14 Hz.
 */
public class UltrasonicPoller extends Thread {
	private static final int THREAD_SLEEP_TIME = 50;
	private SampleProvider us;
	private USController cont;
	public float[] usData;

	public UltrasonicPoller(SampleProvider us,  USController cont) {
		this.us = us;
		this.cont = cont;
		this.usData = new float[us.sampleSize()];
	}

	/**
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 */
	public void run() {
		int distance;
		while (true) {
			us.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			cont.processUSData(distance); // now take action depending on value
			try {
				Thread.sleep(THREAD_SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}
