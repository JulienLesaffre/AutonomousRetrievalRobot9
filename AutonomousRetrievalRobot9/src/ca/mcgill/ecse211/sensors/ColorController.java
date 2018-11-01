package ca.mcgill.ecse211.sensors;


public class ColorController {

	public static float[] ringData;
	
	public void processRingData(float[] data) {
		ringData = data;
	}
}
