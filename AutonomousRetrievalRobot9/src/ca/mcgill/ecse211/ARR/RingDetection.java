package ca.mcgill.ecse211.ARR;

import java.util.ArrayList;

import lejos.robotics.SampleProvider;


/**
 * Class controlling the algorithm for detecting the correct ring color.
 *
 */
public class RingDetection {


	private static SampleProvider colorSample;
	private static float[] rgbValues = new float[3];

	/**
	 * Constructor
	 * @param sp The sample provider for the color detection sensor.
	 */
	public RingDetection(SampleProvider sp) {
		colorSample = sp;
	}
	
	/**
	 * Main logic used to detect the specific ring color. Takes in the RGB values detected
	 * and calculates the Blue/Green ratio and Green/Red ratio as percentages. This method was 
	 * created as a result of testing the sensor.
	 * @param rgb
	 * @return
	 */
	public static int detectColorRatios(float[] rgb) {
		double B_G, G_R;
		B_G = rgb[2]/rgb[1] * 100.0;
		G_R = rgb[1]/rgb[0] * 100.0;
		
		if(B_G > 80.0) {							//if the B/G ratio is above 80 its definitely blue
			return 1;	
		} else if (G_R > 100.0 && G_R <400.0) {		//else if the G/R ratio is larger than 100 but smaller than 400 its definitely green
			return 2; 								
		} else if (G_R > 47.0) {					//if the G/R is between 100 and 47 its yellow
			return 3;
		} else if (G_R <= 47.0 && G_R > 0.0) {		//else its orange
			return 4;
		} else {
			return 0;
		}
	}
	
	/**
	 * This method checks to see if there is a ring ahead. It does so by checking if the intensity levels of
	 * 2 or more of the RGB values is above a certain threshold.
	 * @return True if there is a ring ahead, false otherwise.
	 */
	public static boolean isThereARing() {
		ArrayList<Float> red = new ArrayList<Float>(40);
		ArrayList<Float> green = new ArrayList<Float>(40);
		ArrayList<Float> blue = new ArrayList<Float>(40);
		
		float[] rgbValues = new float[3];
		RingDetection.colorSample.fetchSample(rgbValues, 0);
		
		for(int i = 0; i < 40; i++) {
			red.add(i, rgbValues[0]);
			green.add(i, rgbValues[1]);
			blue.add(i, rgbValues[2]);
		}
		int count = 0;
		
		//if there are 2 or more rings above 0.003 then there is a ring there
		if(averageFloat(red) > 0.0030f) count++;
		if(averageFloat(green) > 0.0030f) count++;
		if(averageFloat(blue) > 0.0030f) count++;
		
		if(count >= 2) 
			return true;
		else
			return false;
	}
	
	//find the average of an array of floats
	public static float averageFloat(ArrayList<Float> samples) {
		float sum = 0;
		for(Float sample : samples)
		    sum += sample;
		return (sum/samples.size());
	}

	/**
	 * Method used to poll the light sensor and return the result
	 * @return The result of the poll. 1 - blue, 2 - green, 3 - yellow, 4 - orange
	 */
	public static int colorDetection() {
		RingDetection.colorSample.fetchSample(rgbValues, 0);
		return RingDetection.detectColorRatios(rgbValues);
	}
	

}