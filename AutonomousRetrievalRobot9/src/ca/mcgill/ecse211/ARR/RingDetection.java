package ca.mcgill.ecse211.ARR;

import java.util.ArrayList;

import lejos.robotics.SampleProvider;

public class RingDetection {


	private static SampleProvider colorSample;
	private static float[] rgbValues = new float[3];

	public RingDetection(SampleProvider sp) {
		colorSample = sp;
	}
	
	
	public static int detectColorRatios(float[] rgb) {
		double B_G, G_R;
		B_G = rgb[2]/rgb[1] * 100.0;
		G_R = rgb[1]/rgb[0] * 100.0;
		
		if(B_G > 80.0) {
			return 1;		//definitely blue
		} else if (G_R > 100.0 && G_R <400.0) {
			return 2; 		//definitely green
		} else if (G_R > 47.0) {
			return 3;
		} else if (G_R <= 47.0 && G_R > 0.0) {
			return 4;
		} else {
			return 0;
		}

		
	}
	
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
	
	public static float averageFloat(ArrayList<Float> samples) {
		float sum = 0;
		for(Float sample : samples)
		    sum += sample;
		return (sum/samples.size());
	}

	public static int colorDetection() {
		RingDetection.colorSample.fetchSample(rgbValues, 0);
		return RingDetection.detectColorRatios(rgbValues);
	}
	
	
	
	

}