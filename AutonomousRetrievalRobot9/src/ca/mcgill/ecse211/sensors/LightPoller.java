package ca.mcgill.ecse211.sensors;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {

	private SampleProvider lg;
	private DataController dataCont;
	private float[] lightData;

	private static int SLEEP_TIME = 50;

	public LightPoller(SampleProvider lg, float[] lightData, DataController dataCont) {
		this.lg = lg;
		this.dataCont = dataCont;
		this.lightData = lightData;
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run() {
		int light;
		while (true) {
			lg.fetchSample(lightData, 0); // acquire data
			light = (int) (lightData[0] * 100); // extract from buffer, cast to int
			dataCont.setL(light); // now take action depending on value
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}
