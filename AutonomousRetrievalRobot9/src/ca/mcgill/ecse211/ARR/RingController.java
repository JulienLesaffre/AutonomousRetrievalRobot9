package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

import java.util.ArrayList;


import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingController {

	//class variables
	private static EV3LargeRegulatedMotor dumpRingMotor;
	private static EV3MediumRegulatedMotor clawMotor;

	
	
	//speed and acceleration
	private static final int COLOR_DETECTION_SPEED = 190;
	private static final int COLOR_DETECTION_ACCEL = 1000;
	private static final int RING_PICKUP_SPEED = 230;
	private static final int RING_PICKUP_ACCEL = 1700;
	private static final int CLAW_GRAB_SPEED = 250;
	private static final int CLAW_GRAB_ACCEL = 6000;
	
	//distance and angles
	private static final int CLAW_GRAB_ANGLE_FULL = 100;
	private static final int CLAW_GRAB_ANGLE_HALF = 135;
	private static final int TOPRING_DETECTION_ANGLE = 75;
	private static final int BOTTOMRING_DETECTION_ANGLE = 39;
	private static final double RING_GRAB_DISTANCE = 6.5;
	private static final int RING_GRAB_ANGLE = 8;
	private static final int COLOR_SAMPLE_SIZE = 300;
	private static final double RING_DETECTION_DISTANCE_TOP = 4.7;
	private static final double RING_DETECTION_DISTANCE_BOTTOM = 6.0;
	
	
	
	/**
	 * Constructor for class, takes in all motor and sensor variables the class uses.
	 * @param odo	odometer
	 * @param lMotor	left motor
	 * @param rMotor	right motor
	 * @param pMotor	pole motor
	 * @param cMotor	claw motor
	 * @param left		left color sensor
	 * @param right		right color sensor
	 * @throws OdometerExceptions
	 */
	public RingController(EV3LargeRegulatedMotor dumpMotor, EV3MediumRegulatedMotor cMotor) throws OdometerExceptions {
		dumpRingMotor = dumpMotor;
		clawMotor = cMotor;
	}
	
	public static void pickUpRings() throws OdometerExceptions {
		//expects to be on grid intersection of one of four sides and facing it
		grabRing();
		for(int i = 0; i < 3; i++) {
			Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(false, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
			grabRing();
		}
		
	}
	
	public static void grabRing() throws OdometerExceptions {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		Navigation.moveStraight(RING_GRAB_DISTANCE, true, false);
		Navigation.turnRobot(RING_GRAB_ANGLE, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		clawMotor.setAcceleration(CLAW_GRAB_ACCEL);
		clawMotor.setSpeed(CLAW_GRAB_SPEED);
		clawMotor.rotateTo(CLAW_GRAB_ANGLE_HALF);
		clawMotor.stop(true);
		Navigation.moveStraight(2, false, false);
		clawMotor.rotateTo(CLAW_GRAB_ANGLE_FULL);
		Navigation.turnRobot(RING_GRAB_ANGLE, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		Navigation.moveStraight(RING_GRAB_DISTANCE, false, false);
		clawMotor.flt();
	}


	
	public static void detectAllRings() throws OdometerExceptions {
		//turn so ring set is to the left of robot
		int colorDetected = -1;
		Navigation.setupHeadingForDetection(true);	
		Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
		Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
		Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		
		detectTopRings();
		boolean top = true;
		for(int i = 0; i < 4; i++) {
			if(i!=0) {
				Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
			}

			if(top) {
				Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, true, false);
				colorDetected = detectColor();
				if(colorDetected <= 0) 
					tryDetectingAgain();
				if(colorDetected <= 0) {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, false, false);
					detectBottomRings();
					top = false;
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, true, false);
					colorDetected = detectColor();
					if(colorDetected <= 0)
						tryDetectingAgain();
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, false, false);
				} else {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, false, false);
				}
				Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
			} else {
				Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, true, false);
				colorDetected = detectColor();
				if(colorDetected <= 0) 
					tryDetectingAgain();
				if(colorDetected <= 0) {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, false, false);
					detectTopRings();
					top = true;
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, true, false);
					colorDetected = detectColor();
					if(colorDetected <= 0) 
						tryDetectingAgain();
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, true, false);
				} else {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, false, false);
				}
				Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
			}
			
			if(i == 3) //dont move to next grid intersection on last detect so you can pick up rings
				continue;
			
			Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
		}
		
		Navigation.findLineStraight(false, RING_PICKUP_SPEED, RING_PICKUP_ACCEL);
		
		resetArms();
	}
	

	public static int tryDetectingAgain() {
		Navigation.moveStraight(1, true, false);
		Navigation.turnRobot(7, true, false, 100, 1000);
		int colorDetected = 0;
		colorDetected = detectColor();
		Navigation.turnRobot(7, false, false, 100, 1000);
		Navigation.moveStraight(1, false, false);
		return colorDetected;
	}
	
	public static int detectColor() {
		
		int mode = 0;
		//check if there even is a ring
		if(RingDetection.isThereARing()) {
			
			ArrayList<Integer> samples = new ArrayList<Integer>(COLOR_SAMPLE_SIZE);
			
			for(int i = 0; i < COLOR_SAMPLE_SIZE; i++) {
				int colorCode = RingDetection.colorDetection();
				samples.add(i, colorCode);
			}
			
			mode = RingController.findMode(samples);
		} 
		//if 0, then there was no ring
		//if -1, error 
		//else gives the ring
		makeSound(mode);
		System.out.println("color: " + mode);
		return mode;
	}
	
	
	
	/**
	 * Find the average of an arrayList of integers. 
	 * @param samples ArrayList of integers to calculate the average of
	 * @return The average of the integers
	 */
	public static int average(ArrayList<Integer> samples) {
		float sum = 0;
		for(Integer sample : samples)
		    sum += sample;
		return (int) (sum/samples.size());
	}
	
	
	//assumed there are a bunch of readings i.e. 300, if there is colors detected more than 
	//40 times we increment a counter, if the counter has more than 1 number, it is an error and we need to alert this.
	//it returns -1 for error
	public static int findMode(ArrayList<Integer> samples) {
		int [] data = new int[5];
		int numberCount = 0;
		for(Integer sample : samples)
			data[sample]++;
		int mode = 0, temp = data[0];
		for(int i = 1; i < 5; i++){
			if(data[i] > 20) numberCount++;
			if(data[i] > temp){
				mode = i;
				temp = data[i];
			}
		}
		if(numberCount > 1) 
			return -1;				//error
		else 
			return mode;
	}
	
	public static void dropRings() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		for(int i= 0; i < 3; i++) {
			dumpRingMotor.rotateTo(45);
			dumpRingMotor.rotateTo(0);
			dumpRingMotor.rotateTo(45);
			dumpRingMotor.rotateTo(0);
		}
		for(int i= 0; i < 2; i++) {
			dumpRingMotor.rotateTo(65);
			dumpRingMotor.rotateTo(0);
			dumpRingMotor.rotateTo(65);
			dumpRingMotor.rotateTo(0);
		}
	}
	
	public static void detectTopRings() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		dumpRingMotor.rotateTo(TOPRING_DETECTION_ANGLE);
		dumpRingMotor.stop();
		clawMotor.rotateTo(135);
		clawMotor.flt();
		dumpRingMotor.flt();
	}
	
	public static void detectBottomRings() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		dumpRingMotor.rotateTo(BOTTOMRING_DETECTION_ANGLE);
		clawMotor.rotateTo(135);
		clawMotor.flt();
	}
	
	public static void resetArms() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		dumpRingMotor.rotateTo(0);
		dumpRingMotor.flt();
		clawMotor.rotateTo(135);
		clawMotor.flt();
	}
	
	
	/**
	 * Takes in the color detected and beeps the corresponding amount of times.
	 * @param color
	 */
	public static void makeSound(int color) {
		if(color > 0) 
			for (int i = 0; i < color; i++) 
				Sound.beep();
	}
	

}