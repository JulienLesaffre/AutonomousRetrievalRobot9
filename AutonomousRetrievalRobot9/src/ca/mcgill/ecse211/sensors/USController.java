package ca.mcgill.ecse211.sensors;

import ca.mcgill.ecse211.ARR.Display;
import ca.mcgill.ecse211.localizers.UltrasonicLocalizer;

/**
 * Separating the controllers for the different sensors
 * 
 * Instead of implementing the different classes (USLocalizer and later on object avoidance)
 * 	to be controllers, we have one controller that filters bad values as this does not change,
 * 	and so that we do not have to initialize the poller twice 
 * 
 * Class is used to define how to handle the data and to save the data
 */
public class USController {

	public static int distance;
	private int filterControl;
	private static final int FILTER_OUT = 15;

	public void processUSData(int distance) {
		// filter bad values
		if (distance >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
		} else if (distance >= 255) {
			USController.distance = distance;
		} else {
			filterControl = 0;
			USController.distance = distance;
		}

		// Print values for debugging
		if(UltrasonicLocalizer.isUSLocalizing) {
			Display.displayUSLocalization(distance, UltrasonicLocalizer.ALPHA, UltrasonicLocalizer.BETA, UltrasonicLocalizer.FINAL_ANGLE);
		} 
	}
}
