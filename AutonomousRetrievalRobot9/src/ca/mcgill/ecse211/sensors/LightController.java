package ca.mcgill.ecse211.sensors;

import ca.mcgill.ecse211.ARR.Display;
import ca.mcgill.ecse211.localizers.LightLocalizer;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;

public class LightController {

	public static float colorLeft;
	public static float colorRight;
	
	public void processLightData(float left, float right) {
		colorLeft = left;
		colorRight = right;
		if(LightLocalizer.isLightLocalizing) {
			double[] data = {0, 0, 0};
			try {
				data = Odometer.getOdometer().getXYT();
			} catch (OdometerExceptions e) {
				e.printStackTrace();
			}
			Display.displayLightLocalization(colorLeft, colorRight, data[0], data[1], data[2]);
		}
	}
	
}
