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
	private static SampleProvider sampleProvider;
	private static SampleProvider leftSampleProvider;
	private static SampleProvider rightSampleProvider;
	
	//store ring information
	private static Ring ringNorth;
	private static Ring ringEast;
	private static Ring ringSouth;
	private static Ring ringWest;
	
	//speed and acceleration
	private static final int COLOR_DETECTION_SPEED = 100;
	private static final int COLOR_DETECTION_ACCEL = 1000;
	
	//distance and angles
	private static final int LOWER_RING_ANGLE_OFFSET = 60;
	private static final int CLAW_GRAB_ANGLE_FULL = 130;
	private static final int CLAW_GRAB_ANGLE_HALF = 50;
	private static final int POLE_TOPRING_ANGLE = 46;
	private static final int POLE_BOTTOMRING_ANGLE = 67;
	private static final double POLE_JAB_DISTANCE = 6;
	
	
	
	public RingController(Odometer odo, EV3LargeRegulatedMotor lMotor, EV3LargeRegulatedMotor rMotor, 
			EV3LargeRegulatedMotor pMotor, EV3MediumRegulatedMotor cMotor, SampleProvider sp, SampleProvider left, 
			SampleProvider right) throws OdometerExceptions {
		odometer = odo;
		leftMotor = lMotor;
		rightMotor = rMotor;
		poleMotor = pMotor;
		clawMotor = cMotor;
		sampleProvider = sp;
		leftSampleProvider = left;
		rightSampleProvider = right;
	}



	public static void pickUpTwoRings() throws OdometerExceptions {
		raisePole();
		
		for(int i = 2; i > 0; i--) {
			
			Navigation.findLineStraight(true);
			Navigation.moveStraight(3 , true, false); //move by + 3 so pole doesnt hit the ring on the way down
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			Navigation.findLineStraight(true);
			Navigation.moveStraight(Navigation.POLE_CENTER_OFFSET, true, false);
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			
			//now retrieve which ring is here, and act accordingly
			Ring ringAhead = getRingAhead();
			

			
			raisePole();
			clawMotor.rotate(-CLAW_GRAB_ANGLE_FULL);		//extend claw
			
			if(ringAhead.isUp) 								//bring down pole
				poleMotor.rotate(POLE_TOPRING_ANGLE);			
			else 
				poleMotor.rotate(POLE_BOTTOMRING_ANGLE);
			
			poleMotor.stop(true);							//hold pole in place
			
			Navigation.moveStraight(POLE_JAB_DISTANCE, true, false);
			
			if(ringAhead.ringColor == 4) 					//if its orange only do half a grab
				clawMotor.rotate(CLAW_GRAB_ANGLE_HALF);
			else 
				clawMotor.rotate(CLAW_GRAB_ANGLE_FULL);
			
			

			//move back, raise pole and hook claw
			Navigation.moveStraight(POLE_JAB_DISTANCE, false, true);

			if(ringAhead.ringColor == 4) 					//extend claw back to push ring in pole
				clawMotor.rotate(-CLAW_GRAB_ANGLE_HALF, true);
			else 
				clawMotor.rotate(-CLAW_GRAB_ANGLE_FULL, true);
			
			raisePoleWithRings();

			Navigation.findLineStraight(false);
			
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, true, false);
		}
		
	}
	
	public static void testGrab() {
		Navigation.setSpeedAcceleration(40, 500);
		raisePole();
		poleMotor.setSpeed(30);
		poleMotor.rotate(POLE_BOTTOMRING_ANGLE);
		Navigation.moveStraight(7, true, false);
		clawMotor.rotate(CLAW_GRAB_ANGLE_FULL);
		Navigation.moveStraight(7, false, true);
		poleMotor.rotate(-POLE_BOTTOMRING_ANGLE);
		clawMotor.rotate(-CLAW_GRAB_ANGLE_FULL, false);
		raisePoleWithRings();
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

	public static void findLineAhead() throws OdometerExceptions {
		// Track how many lines found by left and right sensor
		int foundLeft = 0;
		int foundRight = 0;
		float[] newColorLeft = {0};
		float oldSampleLeft = 0;
		float[] newColorRight = {0};
		float oldSampleRight = 0;
		int colorDetected = RingDetection.colorDetection();
		Navigation.setSpeedAcceleration(60, 500);
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
			Navigation.moveStraight(1.7, true, false);
			colorDetected = RingDetection.colorDetection();
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
					Navigation.moveStraight(1.7, false, false);
					colorDetected = RingDetection.colorDetection();
					ArrayList<Integer> samples = new ArrayList<Integer>(300);
					for(int i = 0; i<300 ; i++) {
						samples.add(RingDetection.colorDetection());
					}
					
					int avg = average(samples);
					saveRingDetection(true, avg);
					
					makeSound(avg);
					String colorDetectedString = Integer.toString(avg);
					Display.lcd.drawString("Color: " + colorDetectedString, 0, 0);
				}
			}
			Navigation.findLineStraight(true);
		}

		
	}
	
	public static void detectAllRings() throws OdometerExceptions {
		
		for(int i=0; i<4; i++) {
			raisePole();

			Navigation.moveStraight(Navigation.RING_DETECTION_OFFSET, true, false);
			Navigation.turnRobot(Navigation.RIGHT_ANGLE, false, false);
			findLineAhead();
			
			
		}
		

	}
	
	public static void makeSound(int color) {
		switch (color) {
		case 1:
			Sound.beep();
			break;
		case 2:
			Sound.beep();
			Sound.beep();
			break;
		case 3:
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		case 4:
			Sound.beep();
			Sound.beep();
			Sound.beep();
			Sound.beep();
			break;
		}
	}
	
	
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
	
	
	public static void raisePole() {
		//make sure color sensor is at its highest
		poleMotor.setAcceleration(500);
		poleMotor.setSpeed(40);
		poleMotor.setStallThreshold(20, 4);
		poleMotor.rotate(-360, true);
		while(true) {
			if(poleMotor.isStalled()) {
				poleMotor.stop(true);
				break;
			}
		}
	}
	public static void raisePoleWithRings() {
		//make sure color sensor is at its highest
		poleMotor.setAcceleration(500);
		poleMotor.setSpeed(40);
		poleMotor.setStallThreshold(20, 7);
		poleMotor.rotate(-200, true);
		while(true) {
			if(poleMotor.isStalled()) {
				poleMotor.stop(true);
				break;
			}
		}
	}
	

	
	/**
	 * drop the rings stored on the claw
	 */
	public static void dropRings() {
		leftMotor.setSpeed(100);
		rightMotor.setSpeed(100);
		poleMotor.rotate(85);
		leftMotor.rotate(-400, true);
		rightMotor.rotate(-400, true);
		for (int i=0; i<= 5; i++) {
			poleMotor.rotate(10);
			poleMotor.rotate(-10);		
		}
	}
	
	

	
	public static int average(ArrayList<Integer> samples) {
		float sum = 0;
		for(Integer sample : samples)
		    sum += sample;
		return (int) (sum/samples.size());
	}
	/*
	poleMotor.setAcceleration(500);
	poleMotor.setSpeed(70);
	poleMotor.rotate(-45);
	poleMotor.stop();
	ringClawMotor.setAcceleration(500);
	ringClawMotor.setSpeed(50);
	ringClawMotor.rotate(110);
	ringClawMotor.stop();
	*/
	
	/*
	poleMotor.setAcceleration(500);
	poleMotor.setSpeed(40);
	poleMotor.setStallThreshold(10, 2);
	poleMotor.rotate(-180, true);
	while(true) {
		if(poleMotor.isStalled()) {
			poleMotor.stop(true);
			break;
		}
	}
	
	Navigation.setSpeedAcceleration(80, 500);
	LightLocalizer.testClaw();
	
	
	
	
	
	ringClawMotor.rotate(-110);
	poleMotor.rotate(44);
	poleMotor.stop(true);
	Navigation.moveStraight(7, true, false);
	ringClawMotor.rotate(50);
	Navigation.moveStraight(7, false, false);
	*/
	

}