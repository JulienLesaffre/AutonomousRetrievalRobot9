package ca.mcgill.ecse211.ARR;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.robotics.SampleProvider;

import java.util.ArrayList;

import ca.mcgill.ecse211.ARR.Ring.Side;
import ca.mcgill.ecse211.odometer.*;

/**
 * 
 * This class handles the travel to the ring set and the ring pick up.
 *
 */
public class RingController {

	//class variables
	private static Odometer odometer;
	private static EV3LargeRegulatedMotor leftMotor;
	private static EV3LargeRegulatedMotor rightMotor;
	private static EV3LargeRegulatedMotor poleMotor;
	private static EV3MediumRegulatedMotor clawMotor;
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	
	//store ring information
	private static Ring ringNorth;
	private static Ring ringEast;
	private static Ring ringSouth;
	private static Ring ringWest;
	
	//speed and acceleration
	private static final int COLOR_DETECTION_SPEED = 90;
	private static final int COLOR_DETECTION_ACCEL = 500;
	
	//distance and angles
	public static final int CLAW_GRAB_ANGLE_FULL = 130;
	public static final int CLAW_GRAB_ANGLE_HALF = 60;
	private static final int POLE_TOPRING_ANGLE = 45;
	private static final int POLE_BOTTOMRING_ANGLE = 66;
	private static final double POLE_JAB_DISTANCE = 15;
	
	
	
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
	public RingController(Odometer odo, EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor, 
			EV3LargeRegulatedMotor pMotor, EV3MediumRegulatedMotor cMotor, SampleProvider left, 
			SampleProvider right) throws OdometerExceptions {
		odometer = odo;
		leftMotor = lMotor;
		rightMotor = rMotor;
		poleMotor = pMotor;
		clawMotor = cMotor;
		leftSampleProvider = left;
		rightSampleProvider = right;
	}



	public static void pickUpTwoRings() throws OdometerExceptions {
		poleMotor.rotateTo(0);
		
		for(int i = 2; i > 0; i--) {
			
			Navigation.findLineStraight(true);
			Navigation.moveStraight(3 , true, false); //move by + 3 so pole doesnt hit the ring on the way down
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			Navigation.findLineStraight(true);
			Navigation.moveStraight(Navigation.POLE_CENTER_OFFSET, true, false);
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			//now retrieve which ring is here, and act accordingly
			Ring ringAhead = getRingAhead();
			Navigation.moveStraight(Navigation.SENSOR_OFFSET + 4, false, false);
			
			

			poleMotor.rotateTo(0);
			clawMotor.rotateTo(0);		//extend claw
			
			if(ringAhead.isUp) 								//bring down pole
				poleMotor.rotateTo(POLE_TOPRING_ANGLE);			
			else 
				poleMotor.rotateTo(POLE_BOTTOMRING_ANGLE);
			
			
			Navigation.setSpeedAcceleration(50, 500);
//			Navigation.findLineStraight(true);
			

			
			Navigation.moveStraight(POLE_JAB_DISTANCE, true, false);
			
			if(ringAhead.ringColor == 4) 					//if its orange only do half a grab
				clawMotor.rotateTo(CLAW_GRAB_ANGLE_HALF);
			else 
				clawMotor.rotateTo(CLAW_GRAB_ANGLE_FULL);
			
			
			//move back, raise pole and hook claw
			Navigation.moveStraight(POLE_JAB_DISTANCE, false, false);

			clawMotor.rotateTo(0);
			poleMotor.rotateTo(POLE_TOPRING_ANGLE);

			Navigation.findLineStraight(false);
			poleMotor.rotateTo(0);
			
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, true, false);
		}
		
	}
	

	//it gets the side of the ring set you are on
	public static Side getSideOfRingSet() {
		int ringSetX, ringSetY;
		if(Navigation.RedTeam == 9) {
			ringSetX = Navigation.TR_x;
			ringSetY = Navigation.TR_y;
		} else {
			ringSetX = Navigation.TG_x;
			ringSetY = Navigation.TG_y;
		}
		
		double myX = odometer.getXYT()[0] / Navigation.SQUARE_SIZE;
		double myY = odometer.getXYT()[1] / Navigation.SQUARE_SIZE;
		int x = (int) Math.round(myX);
		int y = (int) Math.round(myY);

		if(ringSetX > x) 
			return Side.West;
		else if(ringSetX < x)
			return Side.East;
		else if(ringSetY > y)
			return Side.South;
		else
			return Side.North;
	}
	
	
	
	
	public static Ring getRingAhead() {
		int ringSetX, ringSetY;
		if(Navigation.RedTeam == 9) {
			ringSetX = Navigation.TR_x;
			ringSetY = Navigation.TR_y;
		} else {
			ringSetX = Navigation.TG_x;
			ringSetY = Navigation.TG_y;
		}
		
		double myX = odometer.getXYT()[0] / Navigation.SQUARE_SIZE;
		double myY = odometer.getXYT()[1] / Navigation.SQUARE_SIZE;
		int x = (int) Math.round(myX);
		int y = (int) Math.round(myY);

		if(ringSetX > x) 
			return ringWest;
		else if(ringSetX < x)
			return ringEast;
		else if(ringSetY > y)
			return ringSouth;
		else
			return ringNorth;
	}
	
	
	/**
	 * Assumes the robot is on top of one of the 4 corresponding grid intersections on all sides.
	 * It first directs the robot so that the ringset is to the left of it. Moves by an offset distance 
	 * so that when it turns left, the sensor is the correct distance away from the ring. Detects the ring
	 * whether its on top or bottom, saves the value and continues to the next side to do the same.
	 * @throws OdometerExceptions
	 */
	public static void detectAllRings() throws OdometerExceptions {
		//turn so ring set is to the left of robot
		Navigation.setupHeadingForDetection();
		for(int i=0; i<4; i++) {
			poleMotor.rotateTo(0);
			Navigation.moveStraight(Navigation.RING_DETECTION_OFFSET, true, false);
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			detectRing();
		}
	}

	private static void detectRing() throws OdometerExceptions {
		
		int foundLeft = 0, foundRight = 0;						// Track how many lines found by left and right sensor
		float[] newColorLeft = {0}, newColorRight = {0};		
		float oldSampleLeft = 0, oldSampleRight = 0;
		
		Navigation.setSpeedAcceleration(COLOR_DETECTION_SPEED, COLOR_DETECTION_ACCEL);
		
		leftMotor.forward();
		rightMotor.forward();
		
		boolean ringDetected = false;
		
		while(!ringDetected || leftMotor.isMoving() || rightMotor.isMoving()) {
			ringDetected = RingDetection.startOfRing();
			if(ringDetected)
				break;

					
			// Get color sensor readings
			leftSampleProvider.fetchSample(newColorLeft, 0); // acquire data
			rightSampleProvider.fetchSample(newColorRight, 0); 

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
		
		if(ringDetected) {
			Navigation.moveStraight(1.3, true, false);
			
			ArrayList<Integer> samples = new ArrayList<Integer>(300);
			for(int i = 0; i<300 ; i++) {
				samples.add(RingDetection.colorDetection());
			}
			int avg = average(samples);
			saveRingDetection(true, avg);
			makeSound(avg);
			
			System.out.println("" + avg);
			
		}

		if(foundLeft == 0 || foundRight == 0) {
			Navigation.findLineStraight(true);
		}
		
		if(!ringDetected) {
			Navigation.moveStraight(4, true, false);
			poleMotor.rotate(60);
			Navigation.moveStraight(13, false, true);
			ringDetected = false;
			while(!ringDetected) {
				ringDetected = RingDetection.startOfRing();
				if(ringDetected) {
					Navigation.moveStraight(1.3, false, false);
					
					ArrayList<Integer> samples = new ArrayList<Integer>(300);
					for(int i = 0; i<300 ; i++) {
						samples.add(RingDetection.colorDetection());
					}
					
					int avg = average(samples);
					saveRingDetection(false, avg);
					
					makeSound(avg);
					System.out.println("" + avg);
				}
			}
			Navigation.moveStraight(6, false, false);
			Navigation.findLineStraight(true);
		}
	}
	

	
	
	/**
	 * This method assumed the robot is moving past the rings detecting them. Since the 
	 * color sensor is to the left of the robot, when for example the robot is facing north
	 * then the ringset is to the left of the robot and thus the ring currently being
	 * detected is the east side ring. It takes in the color code and whether the ring is
	 * on top or bottom, and saves the data by creating a Ring instance and assigning it to 
	 * one of the four corresponding class variables.
	 * @param ringIsOnTop Whether the ring is on top
	 * @param colorCode The color code of the ring
	 */
	private static void saveRingDetection(boolean ringIsOnTop, int colorCode) {
		//if robot is facing north, ringset is to west, so its east side
		Side ringSide = Side.Null;
		String robotHeading = Navigation.getCurrentHeading();
		if(robotHeading.equalsIgnoreCase("north")) { 						//heading north, if ringset x smaller than tunnel x it's left
			ringSide = Side.East;
			ringEast = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("east")) {					//heading east, if ringset y is larger than tunnel y it's left
			ringSide = Side.South;
			ringSouth = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("south")) { 				//heading south, if ringset x is larger than tunnel x it's left
			ringSide = Side.West;
			ringWest = new Ring(ringIsOnTop, colorCode, ringSide);
		} else if (robotHeading.equalsIgnoreCase("west")) { 				//heading west, if ringset y is smaller than tunnel y it's left
			ringSide = Side.North;
			ringNorth = new Ring(ringIsOnTop, colorCode, ringSide);
		}
	}
	
	/**
	 * First method called when robot starts, it raises the pole until it stalls then
	 * resets the tachometer count.
	 */
	public static void raisePole() {
		//make sure color sensor is at its highest
		poleMotor.setAcceleration(500);
		poleMotor.setSpeed(40);
		poleMotor.rotate(-360, true);
		while(true) {
			if(poleMotor.isStalled()) {
				poleMotor.stop(true);
				break;
			}
		}
		poleMotor.resetTachoCount();
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
	
	
	public static int findMode(ArrayList<Integer> samples) {
		int [] data = new int[5];
		for(Integer sample : samples)
			data[sample]++;
		int mode = 0, temp = data[1];
		for(int i = 2; i < 5; i++){
			if(data[i] > temp){
				mode = i;
				temp = data[i];
			}
		}
		return mode + 1;
	}
	
	
	/**
	 * Drops the rings by rotating the pole to face downwards and the claw motor to be extended.
	 */
	public static void dropRings() {
		clawMotor.rotateTo(0);
		poleMotor.rotateTo(90);
	}
	
	
	/**
	 * Takes in the color detected and beeps the corresponding amount of times.
	 * @param color
	 */
	public static void makeSound(int color) {
		for (int i = 0; i < color; i++) 
			Sound.beep();
	}
	
	public static void testing() {
		
	}
	

}