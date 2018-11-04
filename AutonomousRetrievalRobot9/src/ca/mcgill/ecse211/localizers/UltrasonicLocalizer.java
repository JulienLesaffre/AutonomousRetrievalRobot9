package ca.mcgill.ecse211.localizers;

import ca.mcgill.ecse211.ARR.Display;
import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.*;
import lejos.robotics.SampleProvider;


/**
 * This class controls the ultrasonic localization part. It assumes the 
 * us sensor is polling when calling the fallingEdge main method of 
 * localizing.
 *
 */
public class UltrasonicLocalizer {

	private static final int USLOC_MOTOR_SPEED = 80;
	private static final int USLOC_MOTOR_ACCELERATION = 1000;
	private static final int D_THRESHHOLD = 30;
	private static final int NOISE_MARGIN = 5;
	public static double ALPHA = 0;
	public static double BETA = 0;
	public static double FINAL_ANGLE = 0;
	private static final int FILTER_OUT = 15;
	private int filterControl;
	public float[] usData;
	public static int distance;
	public static boolean isUSLocalizing = false;	
	
	private static SampleProvider usSample;


	public UltrasonicLocalizer(SampleProvider usSampleProvider) {
		usSample = usSampleProvider;
	}

	public void processUSData(int d) {
		// filter bad values
		if (distance >= 255 && filterControl < FILTER_OUT) {
			filterControl++;
		} else if (distance >= 255) {
			distance = d;
		} else {
			filterControl = 0;
			distance = d;
		}
		// Print values for debugging
		if(UltrasonicLocalizer.isUSLocalizing) {
			Display.displayUSLocalization(distance, ALPHA, BETA, FINAL_ANGLE);
		} 
	}

	/**
	 * Performs falling edge localization
	 * @throws OdometerExceptions 
	 */
	public void fallingEdge() throws OdometerExceptions{
		
		isUSLocalizing = true;
		
		Navigation.setSpeedAcceleration(USLOC_MOTOR_SPEED, USLOC_MOTOR_ACCELERATION);
		
		//Instantiate odometer storage and set theta of odometer to 0
		double[] odometer = {0,0,0};
		boolean isAboveThresh = false;
		double angleCorrection = 0;
		Odometer.getOdometer().setTheta(0);
		
		usSample.fetchSample(usData, 0); // acquire data
		distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int

		// Checks orientation or sets orientation to perform localization
		if (distance > (D_THRESHHOLD + NOISE_MARGIN)) {
			isAboveThresh = true;
		} else {
			findWallAbove();
			isAboveThresh = true;
		}

		// Find first falling edge
		while (true) {
			
			usSample.fetchSample(usData, 0); // acquire data
			processUSData((int) (usData[0] * 100.0)); // extract from buffer, cast to int, process data
			
			// Move forward and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();

			// If is falling and you are above the threshold
			// then store theta as alpha and stop turning
			if (isFalling(distance) && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				ALPHA = odometer[2];
				isAboveThresh = false;
				break;
			}
		}

		// Find second falling edge
		while (true) {

			usSample.fetchSample(usData, 0); // acquire data
			processUSData((int) (usData[0] * 100.0)); // extract from buffer, cast to int, process data
			
			// Go backwards and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			Navigation.leftMotor.backward();
			Navigation.rightMotor.forward();

			// Set above thresh to true if you are above the threshold 
			if (distance > (D_THRESHHOLD + NOISE_MARGIN)) {
				isAboveThresh = true;
			}

			// If is falling and you are above the threshold
			// then store 180-theta as beta and stop turning
			if (isFalling(distance) && isAboveThresh) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				BETA = odometer[2];
				break;
			}
		}

		// Alpha and Beta algorithms
		if (ALPHA < BETA) {
			angleCorrection = 40 - ((ALPHA + BETA) / 2); 
		} else {
			angleCorrection = 220 - ((ALPHA + BETA) / 2);
		} 

		// Set theta to 0 to apply correction
		// from current reference angle
		//Odometer.getOdometer().setTheta(0);
		FINAL_ANGLE = 180-(angleCorrection + odometer[2]);
		Navigation.turnTo(FINAL_ANGLE);

		isUSLocalizing = false;
	}


	/**
	 * Sets orientation of robot so it can perform the localization with falling edges 
	 * Makes sure that you are above detectable threshold (i.e facing far from wall)
	 * before you read for falling edge
	 */
	void findWallAbove() {
		while (true) {
			Navigation.leftMotor.forward();
			Navigation.rightMotor.backward();
			if (distance > (D_THRESHHOLD + NOISE_MARGIN)) {
				Navigation.leftMotor.stop(true);
				Navigation.rightMotor.stop(false);
				break;
			}
		}
	}


	/**
	 * Checks if fallingEdge, i.e distance
	 * drops below the threshold
	 * @return boolean: if fallingEdge or not
	 */
	boolean isFalling(int d) {
		if (d < (D_THRESHHOLD - NOISE_MARGIN)) {
			return true;
		} else {
			return false;
		}
	}

}