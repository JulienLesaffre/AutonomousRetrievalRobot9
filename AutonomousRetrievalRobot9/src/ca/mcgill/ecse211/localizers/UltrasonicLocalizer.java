package ca.mcgill.ecse211.localizers;


import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;


/**
 * This class controls the ultrasonic localization part. It assumes the 
 * us sensor is polling when calling the fallingEdge main method of 
 * localizing.
 *
 */
public class UltrasonicLocalizer {

	//constants
	private static final int USLOC_MOTOR_SPEED = 170;
	private static final int USLOC_MOTOR_ACCELERATION = 2000;
	private static final int D_THRESHHOLD = 30;
	private static final int NOISE_MARGIN = 5;
	private static final int FILTER_OUT = 15;
	
	//us sensor variables
	private int filterControl;
	private float[] usData;
	private static int distance;
	
	//sensors and motors
	private static SampleProvider usSample;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor leftMotor;
	private static Odometer odo;
	
	//localizing variables
	private static double ALPHA = 0;
	private static double BETA = 0;
	public static boolean isUSLocalizing = false;	


	/**
	 * Class constructor
	 * @param usSampleProvider The ultrasonic sample provider
	 * @param lMotor Left motor instance
	 * @param rMotor Right motor instance
	 */
	public UltrasonicLocalizer(Odometer odometer, SampleProvider usSampleProvider, EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor) {
		usSample = usSampleProvider;
		usData = new float[usSample.sampleSize()];
		leftMotor = lMotor;
		rightMotor = rMotor;
		odo = odometer;
		
	}

	/**
	 * This method is used to process the data from the ultrasonic sensor. It filters
	 * out the very large values.
	 * @param d
	 */
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
//		double angleCorrection = 0;
		Odometer.getOdometer().setTheta(0);
		
		
		float sum = 0;
		for(int i=0; i < 100; i++) {
			usSample.fetchSample(usData, 0); // acquire data
			distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int
			sum += distance;
		}
		if(sum/100 < (D_THRESHHOLD + NOISE_MARGIN)) {
			findWallAbove();
			isAboveThresh = true;
		} else {
			isAboveThresh = true;
		}
		

		
		usSample.fetchSample(usData, 0); // acquire data
		distance = (int) (usData[0] * 100.0); // extract from buffer, cast to int



		// Find first falling edge
		while (true) {
			
			usSample.fetchSample(usData, 0); // acquire data
			processUSData((int) (usData[0] * 100.0)); // extract from buffer, cast to int, process data
			
			// Move forward and get odometer data
			odometer = Odometer.getOdometer().getXYT();
			leftMotor.forward();
			rightMotor.backward();

			// If is falling and you are above the threshold
			// then store theta as alpha and stop turning
			if (isFalling(distance) && isAboveThresh) {
				leftMotor.stop(true);
				rightMotor.stop(false);
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
			leftMotor.backward();
			rightMotor.forward();

			// Set above thresh to true if you are above the threshold 
			if (distance > (D_THRESHHOLD + NOISE_MARGIN)) {
				isAboveThresh = true;
			}

			// If is falling and you are above the threshold
			// then store 180-theta as beta and stop turning
			if (isFalling(distance) && isAboveThresh) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				BETA = odometer[2];
				break;
			}
		}

		double dTheta = 225.0 - ((ALPHA+BETA)/2.0);
		correctAngle(dTheta);
		
		
//		// Alpha and Beta algorithms
//		if (ALPHA < BETA) {
//			angleCorrection = 40 - ((ALPHA + BETA) / 2); 
//		} else {
//			angleCorrection = 220 - ((ALPHA + BETA) / 2);
//		} 
//
//		// Set theta to 0 to apply correction
//		// from current reference angle
//		//Odometer.getOdometer().setTheta(0);
//		FINAL_ANGLE = 180-(angleCorrection + odometer[2]);
//		Navigation.turnTo(FINAL_ANGLE);

		isUSLocalizing = false;
	}

	
	public void correctAngle(double dTheta) throws OdometerExceptions {
		Odometer odo = Odometer.getOdometer();
		double newTheta = (odo.getXYT()[2] + dTheta) % 360;
		odo.setTheta(newTheta);
		int turnAngle = (int) (360.0 - (newTheta));
		Navigation.turnRobot(turnAngle, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
	}


	/**
	 * Sets orientation of robot so it can perform the localization with falling edges 
	 * Makes sure that you are above detectable threshold (i.e facing far from wall)
	 * before you read for falling edge
	 */
	void findWallAbove() {
		int count = 0;
		while (true) {
			usSample.fetchSample(usData, 0); // acquire data
			processUSData((int) (usData[0] * 100.0)); // extract from buffer, cast to int, process data
			leftMotor.forward();
			rightMotor.backward();
			if(distance > 100)
				count++;
			if (count >= 20) {
				leftMotor.stop(true);
				rightMotor.stop(false);
				leftMotor.resetTachoCount();
				rightMotor.resetTachoCount();
				// Reset the values of x, y and z to 0
				odo.setXYT(0, 0, 0);

				odo.leftMotorTachoCount = 0;
				odo.rightMotorTachoCount = 0;

				// added by me
				odo.lastLeftMotorTachoCount = 0;
				odo.lastRightMotorTachoCount = 0;
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