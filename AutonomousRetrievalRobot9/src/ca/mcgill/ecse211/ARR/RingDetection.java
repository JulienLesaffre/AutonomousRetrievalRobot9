package ca.mcgill.ecse211.ARR;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class RingDetection {

	private static float[] blueRGB = { 0.030f, 0.105f, 0.120f };
	private static float[] greenRGB = { 0.043f, 0.095f, 0.020f };
	private static float[] yellowRGB = { 0.188f, 0.132f, 0.032f };
	private static float[] orangeRGB = { 0.103f, 0.030f, 0.028f };

	public static SensorModes colorSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
	public static SampleProvider colorSample = colorSensor.getMode("RGB");
	float[] colorData;

	public RingDetection() {
		colorData = new float[colorSensor.sampleSize()];
		float[] rgbValues = new float[3];
		colorSample.fetchSample(rgbValues, 0);
	}

	/**
	 * 
	 * @param rgb:
	 *            rgb values detected by the light sensor
	 * @return: returns the integer corresponding to the color detected (1: blue, 2:
	 *          green, 3: yellow, 4: orange and 0 if no color has been detected)
	 */
	private static int colorDetection(float[] rgb) {
		float[] colorsDistances = new float[5];

		// if this distance is the minimum then, no Color/Object is detected
		colorsDistances[0] = 0.15f; // to modify (0.07)

		// distance from Blue
		colorsDistances[1] = (float) distance(blueRGB, rgb);

		// distance from Green
		colorsDistances[2] = (float) distance(greenRGB, rgb);

		// distance from Yellow
		colorsDistances[3] = (float) distance(yellowRGB, rgb);

		// distance from Orange
		colorsDistances[4] = (float) distance(orangeRGB, rgb);

		return getMinIndex(colorsDistances);
	}

	/**
	 * 
	 * @param rgbMean: rgb values of the ring
	 * @param rgbValues: rgb values detected by the light sensor
	 * @return: returns the euclidean distance between the two parameters
	 */
	private static double distance(float[] rgbMean, float[] rgbValues) {
		return Math.sqrt(Math.pow(rgbMean[0] - rgbValues[0], 2) + Math.pow(rgbMean[1] - rgbValues[1], 2)
				+ Math.pow(rgbMean[2] - rgbValues[2], 2));
	}

	/**
	 * 
	 * 
	 * @param inputArray
	 * @return: the index of the minimum element of the array
	 */
	private static int getMinIndex(float[] inputArray) {
		double minValue = inputArray[0];
		int minIndex = 0;
		for (int i = 0; i < inputArray.length; i++) {
			if (inputArray[i] < minValue) {
				minValue = inputArray[i];
				minIndex = i;
			}
		}
		return minIndex;
	}

	/**
	 * 
	 * @return: return the int of color detected (0: None, 1: Blue, 2: Green, 3:
	 *          Yellow, 4: Orange)
	 */
	public static int colorDetection() {
		float[] rgbValues = new float[3];
		RingDetection.colorSample.fetchSample(rgbValues, 0);
//		System.out.println(rgbValues[0] + ";" + rgbValues[1] + ";" + rgbValues[2] );

		return RingDetection.colorDetection(rgbValues);

	}
}