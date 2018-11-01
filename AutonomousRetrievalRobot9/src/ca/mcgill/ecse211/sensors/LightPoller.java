package ca.mcgill.ecse211.sensors;

import lejos.robotics.SampleProvider;

public class LightPoller extends Thread {

	private SampleProvider leftSampleProvider;
	private SampleProvider rightSampleProvider;
	private LightController cont;
	private float[] colorLeft;
	private float[] colorRight;
	private static int SLEEP_TIME = 50;

	public LightPoller(SampleProvider leftSampleProvider, SampleProvider rightSampleProvider, LightController lightCont) {
		this.leftSampleProvider = leftSampleProvider;
		this.rightSampleProvider = rightSampleProvider;
		colorLeft = new float[leftSampleProvider.sampleSize()];
		colorRight = new float[rightSampleProvider.sampleSize()];
		this.cont = lightCont;
	}


	/*
	 * Sensors now return floats using a uniform protocol. Need to convert US result to an integer
	 * [0,255] (non-Javadoc)
	 * 
	 * @see java.lang.Thread#run()
	 */
	public void run(){
		while (true) {
			leftSampleProvider.fetchSample(colorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(colorRight, 0); 
			cont.processLightData(colorLeft[0], colorRight[0]); // now take action depending on value
			try {
				Thread.sleep(SLEEP_TIME);
			} catch (Exception e) {
			} // Poor man's timed sampling
		}
	}
}
