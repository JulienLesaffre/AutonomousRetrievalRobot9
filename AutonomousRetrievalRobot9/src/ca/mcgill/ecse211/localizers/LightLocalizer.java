package ca.mcgill.ecse211.localizers;

import java.util.ArrayList;

import ca.mcgill.ecse211.ARR.Navigation;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import ca.mcgill.ecse211.sensors.LightController;

public class LightLocalizer extends Thread implements Runnable {

	// Instantiate Color sensor and other variables
	static double newColorLeft;
	static double oldSampleLeft;
	static double newColorRight;
	static double oldSampleRight;


	private static Odometer odo;
	private static final int LIGHTLOC_MOTOR_SPEED = 100;
	private static final int LIGHTLOC_MOTOR_ACCELERATION = 1000;
	private static final int RIGHT_ANGLE = 90;
	public static boolean isLightLocalizing = false;
	static ArrayList<Double> points = new ArrayList<Double>();
	double[] oldResult = new double [3];
	static int passedLine;
	static double xOffset = 0;
	static double yOffset = 0;
	double dy;
	double dx;

	
	public LightLocalizer() throws OdometerExceptions {
		odo = Odometer.getOdometer(Navigation.leftMotor, Navigation.rightMotor, Navigation.TRACK, Navigation.WHEEL_RAD);
	}

	
	public void localize(int startingCorner) throws OdometerExceptions {
		
		isLightLocalizing = true;
		
		Navigation.setSpeedAcceleration(LIGHTLOC_MOTOR_SPEED, LIGHTLOC_MOTOR_ACCELERATION);
		
		// Move to the line in front 
		findLineAhead();

		// Move to the line to the right 
		findRightLine();
		
		//correct the odometer 
		correctOdometer(startingCorner);
		
		isLightLocalizing = false;
		
	}

	
	private void correctOdometer(int startingCorner) {
		//Correct the odometer depending on the starting corner
		switch(startingCorner) {
		case 0:
			odo.setX(Navigation.SQUARE_SIZE);
			odo.setY(Navigation.SQUARE_SIZE);
			odo.setTheta(0);
			break;
		case 1:
			odo.setX(14.0 * Navigation.SQUARE_SIZE);
			odo.setY(Navigation.SQUARE_SIZE);
			odo.setTheta(270);
			break;
		case 2:
			odo.setX(14.0 * Navigation.SQUARE_SIZE);
			odo.setY(8.0 * Navigation.SQUARE_SIZE);
			odo.setTheta(180);
			break;
		case 3:
			odo.setX(Navigation.SQUARE_SIZE);
			odo.setY(8.0 * Navigation.SQUARE_SIZE);
			odo.setTheta(90);
			break;
		}
	}
	

	public static void findRightLine() {
		
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;

		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();

		while(true) {
			// Get color sensor readings
			newColorLeft = LightController.colorLeft;
			newColorRight = LightController.colorRight;

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				Navigation.leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				Navigation.rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft;
			oldSampleRight = newColorRight;

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

	
	
	public static void findLineAhead() throws OdometerExceptions {
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;


		Navigation.leftMotor.forward();
		Navigation.rightMotor.forward();

		while(true) {
			//color sensor and scaling
			newColorLeft = LightController.colorLeft;
			newColorRight = LightController.colorRight;

			// If line detected for left sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorLeft) < 0.3 && oldSampleLeft > 0.3 && foundLeft == 0) {
				Navigation.leftMotor.stop(true);
				foundLeft++;
			}
			// If line detected for right sensor (intensity less than 0.3), only count once by keeping track of last value
			if((newColorRight) < 0.3 && oldSampleRight > 0.3 && foundRight == 0) {
				Navigation.rightMotor.stop(true);
				foundRight++;
			}

			// Store last color readings
			oldSampleLeft = newColorLeft;
			oldSampleRight = newColorRight;

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
	
}