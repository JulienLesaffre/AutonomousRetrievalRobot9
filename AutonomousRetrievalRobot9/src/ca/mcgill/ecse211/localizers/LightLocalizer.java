package ca.mcgill.ecse211.localizers;

import ca.mcgill.ecse211.ARR.Display;
import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

/**
 * This class performs the light localization at start of game.
 *
 */
public class LightLocalizer {

	//sensor variables
	static float[] newColorLeft;
	static float oldSampleLeft;
	static float[] newColorRight;
	static float oldSampleRight;

	//sensors and motors
	private static SampleProvider leftSample;
	private static SampleProvider rightSample;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor leftMotor;
	
	//constants
	private static final int LIGHTLOC_MOTOR_SPEED = 170;
	private static final int LIGHTLOC_MOTOR_ACCELERATION = 1500;
	private static final int RIGHT_ANGLE = 90;
	
	//associations
	private static Odometer odo;
	
	
	public static boolean isLightLocalizing = false;


	/**
	 * Class constructor
	 * @param left Left light sensor sample provider
	 * @param right Right light sensor sample provider
	 * @param odometer Odometer updating the position of the robot
	 * @param lMotor Left motor instance
	 * @param rMotor Right motor instance
	 * @throws OdometerExceptions
	 */
	public LightLocalizer(SampleProvider left, SampleProvider right, Odometer odometer, EV3LargeRegulatedMotor lMotor,
			EV3LargeRegulatedMotor rMotor) throws OdometerExceptions {
		odo = odometer;
		leftSample = left;
		rightSample = right;
		newColorLeft = new float[leftSample.sampleSize()];
		newColorRight = new float[rightSample.sampleSize()];
		leftMotor = lMotor;
		rightMotor = rMotor;
	}

	
	/**
	 * This method assumes the ultrasonic localization has finished and robot is 
	 * in the middle of the starting tile. It first moves forward and stops robot 
	 * with light sensors ontop of perpendicular line ahead. Moves by offset of 
	 * center to light sensors, turns right and does the same for the next perpendicular
	 * line. Once done corrects the odometer readings.
	 * 
	 * @param startingCorner : starting corner of robot, main must pass this in
	 * @throws OdometerExceptions
	 */
	public void localize() throws OdometerExceptions {
		
		isLightLocalizing = true;
		
		Navigation.setSpeedAcceleration(LIGHTLOC_MOTOR_SPEED, LIGHTLOC_MOTOR_ACCELERATION);
		
		// Move to the line in front 
		findLineAhead();

		// Move to the line to the right 
		findRightLine();
		
		//correct the odometer based on starting corner
		if(Navigation.GreenTeam == 9) {
			correctOdometer(Navigation.GreenCorner);
		} else {
			correctOdometer(Navigation.RedCorner);
		}
		
		
		double[] data = odo.getXYT();
		Display.displayNavigation(data[0], data[1], data[2]);
		
		Sound.beep();
		Sound.beep();
		Sound.beep();
		
		isLightLocalizing = false;
		
	}

	
	/**
	 * Based on starting corner, the same localization technique is performed
	 * by going forward to first line then right to second line. Using this 
	 * information and the size of the field we can correct the odometer readings.
	 * 
	 * @param startingCorner The starting corner of the robot from 0-3
	 */
	private void correctOdometer(int startingCorner) {
		//Correct the odometer depending on the starting corner
		switch(startingCorner) {
		case 0:
			odo.setX(Navigation.SQUARE_SIZE);
			odo.setY(Navigation.SQUARE_SIZE);
			odo.setTheta(0);
			break;
		case 1:
			odo.setX(7.0 * Navigation.SQUARE_SIZE);
			odo.setY(Navigation.SQUARE_SIZE);
			odo.setTheta(270);
			break;
		case 2:
			odo.setX(7.0 * Navigation.SQUARE_SIZE);
			odo.setY(7.0 * Navigation.SQUARE_SIZE);
			odo.setTheta(180);
			break;
		case 3:
			odo.setX(Navigation.SQUARE_SIZE);
			odo.setY(7.0 * Navigation.SQUARE_SIZE);
			odo.setTheta(90);
			break;
		}
	}
	

	/**
	 * This method finds the second right line for correcting the odometer
	 */
	public static void findRightLine() {
		
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;

		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			// Get color sensor readings
			leftSample.fetchSample(newColorLeft, 0); // acquire data
			rightSample.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}

		// Move forward by length of offset
		Navigation.moveStraight(Navigation.SENSOR_OFFSET, true, false);
		// Turn left 90 degrees to face 0 degrees
		Navigation.turnRobot(RIGHT_ANGLE, false, false);
		
	}

	
	
	/**
	 * This method find the first line ahead when starting the localization.
	 * @throws OdometerExceptions
	 */
	public static void findLineAhead() throws OdometerExceptions {
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;


		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			// Get color sensor readings
			leftSample.fetchSample(newColorLeft, 0); // acquire data
			rightSample.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}

		// Move forward by length of offset
		Navigation.moveStraight(Navigation.SENSOR_OFFSET, true, false);
		// Turn left 90 degrees to face 0 degrees
		Navigation.turnRobot(RIGHT_ANGLE, true, false);
		
	}
	
	
	
	/**
	 * This method finds the second right line for correcting the odometer
	 */
	public static void findright() {
		
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;

		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			// Get color sensor readings
			leftSample.fetchSample(newColorLeft, 0); // acquire data
			rightSample.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}

		
	}

	
	
	/**
	 * This method find the first line ahead when starting the localization.
	 * @throws OdometerExceptions
	 */
	public static void findstraight() throws OdometerExceptions {
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;


		leftMotor.forward();
		rightMotor.forward();

		while(true) {
			// Get color sensor readings
			leftSample.fetchSample(newColorLeft, 0); // acquire data
			rightSample.fetchSample(newColorRight, 0); 

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft[0]) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight[0]) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft[0];
			oldSampleRight = newColorRight[0];

			// If line found for both sensors, exit
			if(foundLeft == 1 && foundRight == 1) {
				break;
			}
		}

		// Move forward by length of offset
		Navigation.moveStraight(1.5, true, false);
		Navigation.turnRobot(RIGHT_ANGLE, true, false);
		
	}
	
}