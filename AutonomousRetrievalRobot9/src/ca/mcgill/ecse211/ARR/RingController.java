package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;

import java.util.ArrayList;


import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles detecting the colors of the ringset and picking up the rings.
 *
 */
public class RingController {

	//class variables
	private static EV3LargeRegulatedMotor dumpRingMotor;
	private static EV3MediumRegulatedMotor clawMotor;

	//speed and acceleration
	private static final int COLOR_DETECTION_SPEED = 240;
	private static final int COLOR_DETECTION_ACCEL = 1400;
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
	
	public static boolean shouldWeTurn = false;
	
	
	/**
	 * Constructor for class, takes in all motor variables the class uses
	 * @param dumpMotor The motor controlling the arms with the light sensor
	 * @param cMotor The motor controlling the claw grabbing mechanism
	 * @throws OdometerExceptions
	 */
	public RingController(EV3LargeRegulatedMotor dumpMotor, EV3MediumRegulatedMotor cMotor) throws OdometerExceptions {
		dumpRingMotor = dumpMotor;
		clawMotor = cMotor;
	}
	
	
	/**
	 * This method makes the robot circulate the ringset grabbing the rings. The robot localizes on both axis
	 * centering the robot directly infront of the side of the ringset, extends the claw, moves forward, rotates the claw and
	 * moves backward grabbing the ring then moves on to the next side. Asssumes the robot to already be on one of the 4 adjacent
	 * grid intersections.
	 * @throws OdometerExceptions
	 */
	public static void pickUpRings() throws OdometerExceptions {
		//grab the first ring
		grabRing();
		double[] tunnelMidpointEnd = Navigation.findTunnelMidpoint(false);
		//loop 3 times to grab the remaining rings
		for(int i = 0; i < 3; i++) {
			//condition to check for the case that the movement will collide with the edge of the tunnel
			//moves by 60 degrees first to avoid the edge
			if(!Navigation.isRingSetToLeft() && i == 1 && Navigation.hasArrived(Navigation.targetCoordinate[0], Navigation.targetCoordinate[1], 5)) {
				if(Navigation.hasArrived(tunnelMidpointEnd[0], tunnelMidpointEnd[1], (int)(Navigation.SQUARE_SIZE*1.5))) {
					Navigation.turnRobot(60, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
					Navigation.moveStraight(12, true, false);
				}	else {
					Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				}
			} else if(Navigation.isRingSetToLeft() && i == 0 && Navigation.hasArrived(Navigation.targetCoordinate[0], Navigation.targetCoordinate[1], (int)(Navigation.SQUARE_SIZE*2))) {
				if(Navigation.hasArrived(tunnelMidpointEnd[0], tunnelMidpointEnd[1], (int)(Navigation.SQUARE_SIZE*1.75))) {
					shouldWeTurn = true;
					Navigation.turnRobot(60, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
					Navigation.moveStraight(12, true, false);
				}	else {
					Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				}
			} else {
				Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			}
			//move to the next grid intersection
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(false, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
			//grab next ring
			grabRing();
		}
	}
	
	/**
	 * Method used to grab the ring, assumes the robot has localized on both axis and is on one of the
	 * grid intersections adjacent to the ring set and facing the side of the ringset. Extends the claw
	 * moves forward and grabs the ring.
	 * @throws OdometerExceptions
	 */
	public static void grabRing() throws OdometerExceptions {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		Navigation.moveStraight(RING_GRAB_DISTANCE, true, false);
		Navigation.turnRobot(RING_GRAB_ANGLE, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);	//turn by small angle to ensure best grab
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


	/**
	 * This method is called to detect all the rings on the ringset. It circulates the ringset in an
	 * anticlockwise manner and first detects the top ringset, if it does not detect one on the top it switches
	 * the angle to detect the bottom ring. Optimized to continue to next side when ring is detected on either the
	 * top or bottom ringset.
	 * @throws OdometerExceptions
	 */
	public static void detectAllRings() throws OdometerExceptions {
		//turn so ring set is to the left of robot
		int colorDetected = -1;

		Navigation.setupHeadingForDetection(true);	//set up robot so it's heading is towards the ringset
		
		//localize on both lines incase the robot's center is not exactly ontop of grid intersection
		Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL, true);
		Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		Navigation.moveStraight(3, true, false);
		Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL, true);
		Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
		
		//detect the top ring first
		detectTopRings();
		boolean top = true;
		for(int i = 0; i < 4; i++) {
			if(i!=0) {
				Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL, true);
			}
			
			//if detecting top, move by top offset, else bottom offset
			if(top)
				Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, true, false);
			else
				Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, true, false);
			
			//detect the color
			colorDetected = detectColor();
			
			//if no color detected try again with different angle and distance
			if(colorDetected <= 0) 
				tryDetectingAgain();
			
			//if color still not detected, switch from top to bottom or vice versa and try again
			if(colorDetected <= 0) {
				if(top) {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, false, false);
					detectBottomRings();
					top = false;
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, true, false);
				}
				else {
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, false, false);
					detectTopRings();
					top = true;
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, true, false);
				}
				colorDetected = detectColor();
				if(colorDetected <= 0)
					tryDetectingAgain();
				if(top)
					Navigation.moveStraight(RING_DETECTION_DISTANCE_TOP, false, false);
				else
					Navigation.moveStraight(RING_DETECTION_DISTANCE_BOTTOM, false, false);
				
			} 
			
			//move back to grid intersection
			Navigation.findLineStraight(false, COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL, true);

			if(i == 3) //dont move to next grid intersection on last detect so you can pick up rings
				continue;
			
			double[] tunnelMidpointEnd = Navigation.findTunnelMidpoint(false);
			
			//condition to check if when moving to next grid intersection there is the tunnel corner
			if(!Navigation.isRingSetToLeft() && i == 0 && Navigation.hasArrived(Navigation.targetCoordinate[0], Navigation.targetCoordinate[1], 5)) {
				if(Navigation.hasArrived(tunnelMidpointEnd[0], tunnelMidpointEnd[1], (int)(Navigation.SQUARE_SIZE*1.5))) {
					Navigation.turnRobot(60, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
					Navigation.moveStraight(12, true, false);
				} else {
					Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				}
			} else if(Navigation.isRingSetToLeft() && i == 3 && Navigation.hasArrived(Navigation.targetCoordinate[0], Navigation.targetCoordinate[1], (int)(Navigation.SQUARE_SIZE*1.5))) {
				if(Navigation.hasArrived(tunnelMidpointEnd[0], tunnelMidpointEnd[1], (int)(Navigation.SQUARE_SIZE*1.5))) {
					Navigation.turnRobot(60, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
					Navigation.moveStraight(12, true, false);
				} else {
					Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
				}
			} else {
				Navigation.turnRobot(90, true, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			}

			//move to next grid intersection
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
			Navigation.turnRobot(90, false, false, Navigation.ROTATE_SPEED_FAST, Navigation.ROTATE_ACCEL_FAST);
			Navigation.findLineStraight(true, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
		}
		
		Navigation.findLineStraight(false, RING_PICKUP_SPEED, RING_PICKUP_ACCEL, true);
		
		//finally reset arms in preparation for pickup
		resetArms();
	}
	

	/**
	 * This method is used when there is no ring detected, it moves the robot close and to the side
	 * to see if it can get a different angle and better reading. Returns the color detected.
	 * @return Color detected as int.
	 */
	public static int tryDetectingAgain() {
		Navigation.moveStraight(1, true, false);
		Navigation.turnRobot(7, true, false, 100, 1000);
		int colorDetected = 0;
		colorDetected = detectColor();
		Navigation.turnRobot(7, false, false, 100, 1000);
		Navigation.moveStraight(1, false, false);
		return colorDetected;
	}
	
	/**
	 * This method is called when the robot is positioned to detected the color. It first checks
	 * if there is a ring, if there is it takes a bunch of samples, finds the mode and returns the mode
	 * as a result and beeps the number of times of the color detected.
	 * @return The color detected as an int.
	 */
	public static int detectColor() {
		int mode = 0;
		//check if there even is a ring
		if(RingDetection.isThereARing()) {
			ArrayList<Integer> samples = new ArrayList<Integer>(COLOR_SAMPLE_SIZE);
			//take a bunch of readings and add to arraylist
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
		return mode;
	}
	
	
	
	//Find the average of an arrayList of integers. 
	public static int average(ArrayList<Integer> samples) {
		float sum = 0;
		for(Integer sample : samples)
		    sum += sample;
		return (int) (sum/samples.size());
	}
	
	
	/**
	 * Find the mode of an array of integers. Assumes there are many readings taken. If there is a color detected more than 40 times,
	 * we increment a counter, if the counter is larger than 1 there is an error and we need to alert this.
	 * @param samples Array of integers - the samples
	 * @return The mode of the samples, or -1 for error
	 */
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
	
	
	/**
	 * This method uses the arms to push off the rings.
	 */
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
	
	/**
	 * Positions the arms to detect the rings on the top of the ringset.
	 */
	public static void detectTopRings() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		dumpRingMotor.rotateTo(TOPRING_DETECTION_ANGLE);
		dumpRingMotor.stop();
		clawMotor.rotateTo(135);
		clawMotor.flt();
		dumpRingMotor.flt();
	}
	
	/**
	 * Positions the arms to detect the rings on the bottom of the ringset.
	 */
	public static void detectBottomRings() {
		clawMotor.rotateTo(0);
		clawMotor.flt();
		dumpRingMotor.rotateTo(BOTTOMRING_DETECTION_ANGLE);
		clawMotor.rotateTo(135);
		clawMotor.flt();
	}
	
	/**
	 * This method resets the arms of the robot so that two arms are downwards and the claw is clenched.
	 */
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